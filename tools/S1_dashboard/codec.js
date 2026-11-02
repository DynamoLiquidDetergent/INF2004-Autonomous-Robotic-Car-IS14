/*
 * codec.js
 * Subsystem 1 PC dashboard: frame decoder, command/map encoder, self-test.
 *
 * The layout tables below MIRROR app_program/subsystem1/telem_layout.h.
 * If a width or field order changes there, change it here too and bump
 * LAYOUT_VERSION to match TELEM_VERSION. The self-test (golden bytes from
 * S1-U11) fails if the two drift apart.
 *
 * Runs in the browser (window.S1Codec) and in Node:
 *     node codec.js        runs the self-test
 */

(function (root) {
    "use strict";

    /* ---- Header byte (telem_layout.h) -------------------------------- */
    var LAYOUT_VERSION = 1;
    var HDR_VER_MASK = 0x07;
    var HDR_TYPE_SHIFT = 3;
    var HDR_TYPE_MASK = 0x07;
    var HDR_REPLAYED = 0x40;

    var TYPE_TELEM = 0;
    var TYPE_HB = 1;
    var TYPE_EVT = 2;

    /* ---- Field tables, in packing order --------------------------------
     * name, bits, none (raw "no data" value, or null if the field has
     * none), signed (two's complement), unit.                              */
    var TELEM_FIELDS = [
        { name: "time_ds",       bits: 16, none: null, unit: "0.1 s" },
        { name: "mission_state", bits: 4,  none: 15,   unit: "" },
        { name: "fault_mask",    bits: 4,  none: null, unit: "" },
        { name: "speed_left",    bits: 10, none: 512,  unit: "mm/s", signed: true },
        { name: "speed_right",   bits: 10, none: 512,  unit: "mm/s", signed: true },
        { name: "motion_state",  bits: 3,  none: null, unit: "" },
        { name: "dist_mm",       bits: 16, none: null, unit: "mm" },
        { name: "line_zone",     bits: 3,  none: 7,    unit: "" },
        { name: "junction",      bits: 1,  none: null, unit: "" },
        { name: "line_lost",     bits: 1,  none: null, unit: "" },
        { name: "letter",        bits: 5,  none: 31,   unit: "" },
        { name: "last_action",   bits: 2,  none: null, unit: "" },
        { name: "map_ok",        bits: 1,  none: null, unit: "" },
        { name: "barcodes",      bits: 4,  none: null, unit: "" },
        { name: "hump_mm",       bits: 8,  none: 255,  unit: "mm" },
        { name: "obstacle",      bits: 1,  none: null, unit: "" },
        { name: "obst_dist_cm",  bits: 9,  none: 511,  unit: "cm" },
        { name: "obst_side",     bits: 2,  none: null, unit: "" },
        { name: "obst_width_cm", bits: 6,  none: 63,   unit: "cm" }
    ];

    var HB_FIELDS = [
        { name: "seq",           bits: 8, none: null },
        { name: "mission_state", bits: 4, none: 15 },
        { name: "outage_count",  bits: 5, none: null },
        { name: "wifi_up",       bits: 1, none: null },
        { name: "mqtt_up",       bits: 1, none: null },
        { name: "map_ok",        bits: 1, none: null },
        { name: "rejected",      bits: 4, none: null }
    ];

    var EVT_FIELDS = [
        { name: "evt_type", bits: 3,  none: null },
        { name: "data",     bits: 13, none: null }
    ];

    var FRAME_LEN = {};
    FRAME_LEN[TYPE_TELEM] = 15;
    FRAME_LEN[TYPE_HB] = 4;
    FRAME_LEN[TYPE_EVT] = 3;

    var TYPE_NAMES = ["TELEM", "HB", "EVT"];
    var EVT_NAMES = ["BARCODE", "JUNCTION", "MAP_ACK", "CMD_ACK",
                     "HUMP", "OBSTACLE", "STATE_CHANGE", "FAULT"];
    var MOVE_NAMES = ["FORWARD", "U-TURN", "LEFT", "RIGHT"];
    var SIDE_NAMES = ["none", "left", "right", "?"];
    var MAP_REASONS = ["format", "letter", "duplicate", "state"];
    var STATE_NAMES = { 1: "READY" };          /* PROPOSED, IF-13 */

    var CMD_CODES = { PAUSE: 1, RESUME: 2, ABORT: 3 };
    var CMD_NAMES = ["?", "PAUSE", "RESUME", "ABORT"];

    /* ---- Bit reader / writer (LSB first, like telem_pack.c) ---------- */
    function getBits(bytes, pos, width) {
        var value = 0;
        var i;
        for (i = 0; i < width; i++) {
            var p = pos + i;
            if ((bytes[p >> 3] >> (p & 7)) & 1) {
                value += Math.pow(2, i);
            }
        }
        return value;
    }

    function putBits(bytes, pos, value, width) {
        var i;
        for (i = 0; i < width; i++) {
            var p = pos + i;
            if (Math.floor(value / Math.pow(2, i)) % 2) {
                bytes[p >> 3] |= (1 << (p & 7));
            }
        }
    }

    function unpack(body, fields) {
        var out = {};
        var pos = 0;
        fields.forEach(function (f) {
            var raw = getBits(body, pos, f.bits);
            pos += f.bits;
            if (f.none !== null && raw === f.none) {
                out[f.name] = null;                   /* no data yet */
            } else if (f.signed && raw >= Math.pow(2, f.bits - 1)) {
                out[f.name] = raw - Math.pow(2, f.bits);
            } else {
                out[f.name] = raw;
            }
        });
        return out;
    }

    function letterName(code) {
        return (code === null || code > 25) ? "-" : String.fromCharCode(65 + code);
    }

    function stateName(s) {
        if (s === null) { return "-"; }
        return STATE_NAMES[s] ? (s + " " + STATE_NAMES[s]) : String(s);
    }

    /* ---- Event meaning (telemetry.c event data layouts) -------------- */
    function describeEvent(type, data) {
        switch (type) {
        case 0:
            return "barcode " + letterName(data & 0x1F) +
                   (((data >> 5) & 1) ? " (reversed)" : "");
        case 1:
            return "junction " + letterName(data & 0x1F) + " -> " +
                   MOVE_NAMES[(data >> 5) & 3];
        case 2:
            return (data & 1) ? "map accepted" :
                   "map rejected (" + MAP_REASONS[(data >> 1) & 3] + ")";
        case 3:
            return (data & 4) ? (CMD_NAMES[data & 3] + " accepted") :
                   "command rejected";
        case 4:
            return "hump " + (data & 0xFF) + " mm";
        case 5:
            return "obstacle " + (data & 0x3F) + " cm, " +
                   SIDE_NAMES[(data >> 6) & 3];
        case 6:
            return "state " + stateName((data >> 4) & 0x0F) + " -> " +
                   stateName(data & 0x0F);
        case 7:
            return "fault mask 0x" + (data & 0x0F).toString(16);
        default:
            return "?";
        }
    }

    /*
     * Decode one frame (array of byte values).
     * Returns { ok, type, typeName, version, replayed, fields, text }
     * or { ok: false, error } -- never throws.
     */
    function decodeFrame(bytes) {
        if (!bytes || bytes.length < 1) {
            return { ok: false, error: "empty frame" };
        }
        var hdr = bytes[0];
        var version = hdr & HDR_VER_MASK;
        var type = (hdr >> HDR_TYPE_SHIFT) & HDR_TYPE_MASK;
        var replayed = (hdr & HDR_REPLAYED) !== 0;

        if (version !== LAYOUT_VERSION) {
            return { ok: false, error: "layout version " + version +
                     ", dashboard expects " + LAYOUT_VERSION };
        }
        if (FRAME_LEN[type] === undefined) {
            return { ok: false, error: "unknown frame type " + type };
        }
        if (bytes.length !== FRAME_LEN[type]) {
            return { ok: false, error: TYPE_NAMES[type] + " frame is " +
                     bytes.length + " bytes, expected " + FRAME_LEN[type] };
        }

        var body = Array.prototype.slice.call(bytes, 1);
        var res = { ok: true, type: type, typeName: TYPE_NAMES[type],
                    version: version, replayed: replayed };

        if (type === TYPE_TELEM) {
            res.fields = unpack(body, TELEM_FIELDS);
        } else if (type === TYPE_HB) {
            res.fields = unpack(body, HB_FIELDS);
        } else {
            res.fields = unpack(body, EVT_FIELDS);
            res.evtName = EVT_NAMES[res.fields.evt_type];
            res.text = describeEvent(res.fields.evt_type, res.fields.data);
        }
        return res;
    }

    /* ---- Encoders (cmd and map topics, IF-15 RX) --------------------- */

    /* "PAUSE" / "RESUME" / "ABORT" -> { ok, bytes } */
    function encodeCmd(name) {
        var code = CMD_CODES[String(name).toUpperCase()];
        if (!code) {
            return { ok: false, error: "unknown command " + name };
        }
        return { ok: true, bytes: [code] };
    }

    /*
     * Four letters (FORWARD, U-TURN, LEFT, RIGHT) -> 3-byte map frame.
     * Same checks as cmd_parse_map(), except READY (only the car knows).
     */
    function encodeMap(fwd, uturn, left, right) {
        var letters = [fwd, uturn, left, right];
        var codes = [];
        var i;
        for (i = 0; i < 4; i++) {
            var s = String(letters[i] === undefined || letters[i] === null ?
                           "" : letters[i]).trim().toUpperCase();
            if (!/^[A-Z]$/.test(s)) {
                return { ok: false, error: MOVE_NAMES[i] + ": enter one letter A-Z" };
            }
            if (codes.indexOf(s.charCodeAt(0) - 65) >= 0) {
                return { ok: false, error: "letter " + s + " used twice" };
            }
            codes.push(s.charCodeAt(0) - 65);
        }
        var bytes = [0, 0, 0];
        for (i = 0; i < 4; i++) {
            putBits(bytes, i * 5, codes[i], 5);
        }
        return { ok: true, bytes: bytes };
    }

    function toHex(bytes) {
        return Array.prototype.map.call(bytes, function (b) {
            return ("0" + b.toString(16)).slice(-2);
        }).join(" ");
    }

    function fromHex(str) {
        return str.trim().split(/\s+/).map(function (h) { return parseInt(h, 16); });
    }

    /* ---- Self-test ------------------------------------------------------
     * Golden bytes come from the C packer (S1-U11, subsystem1_test.c),
     * so these checks prove this decoder matches the car's layout.       */
    function same(a, b) {
        return JSON.stringify(a) === JSON.stringify(b);
    }

    function selfTest(print) {
        var pass = 0;
        var fail = 0;
        function check(id, cond, desc) {
            if (cond) { pass++; } else { fail++; }
            print("[TEST] " + id + (cond ? " PASS  " : " FAIL  ") + desc);
        }

        /* golden telemetry frame */
        var t = decodeFrame(fromHex("01 d2 04 53 c8 a8 ad 70 08 a5 3c 2a af f4 00"));
        check("S1-U35", t.ok && t.typeName === "TELEM" && !t.replayed && same(t.fields, {
            time_ds: 1234, mission_state: 3, fault_mask: 5, speed_left: 200,
            speed_right: -150, motion_state: 2, dist_mm: 4321, line_zone: 2,
            junction: 1, line_lost: 0, letter: 10, last_action: 2, map_ok: 1,
            barcodes: 3, hump_mm: 42, obstacle: 1, obst_dist_cm: 87,
            obst_side: 1, obst_width_cm: 15 }),
            "decode golden telemetry frame (all 19 fields)");

        /* golden heartbeat and event frames */
        var h = decodeFrame(fromHex("09 c8 43 9b"));
        var e = decodeFrame(fromHex("11 26 01"));
        check("S1-U35", h.ok && same(h.fields, { seq: 200, mission_state: 3,
            outage_count: 20, wifi_up: 1, mqtt_up: 0, map_ok: 1, rejected: 9 }) &&
            e.ok && e.evtName === "STATE_CHANGE" && e.text === "state 2 -> 4",
            "decode golden heartbeat and event frames");

        /* every event seen on the board in U32-U34 */
        var seen = [
            ["11 0e 01", "state 2 -> 1 READY"],
            ["11 96 00", "state 1 READY -> 2"],
            ["11 2b 00", "PAUSE accepted"],
            ["11 03 00", "command rejected"],
            ["11 0a 00", "map accepted"],
            ["11 32 00", "map rejected (state)"]
        ];
        check("S1-U35", seen.every(function (p) {
            var d = decodeFrame(fromHex(p[0]));
            return d.ok && d.text === p[1];
        }), "event text for the 6 events seen on the board");

        /* replayed bit and "no data" values */
        var r = decodeFrame(fromHex("41 d2 04 53 c8 a8 ad 70 08 a5 3c 2a af f4 00"));
        var n = decodeFrame(fromHex("01 ff ff ff ff ff ff ff ff ff ff ff ff ff ff"));
        var sp = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        putBits(sp, 24, 512, 10);                    /* speed_left = NONE */
        var s = decodeFrame([0x01].concat(sp));
        check("S1-U35", r.ok && r.replayed && same(r.fields, t.fields) &&
            n.ok && n.fields.mission_state === null && n.fields.line_zone === null &&
            n.fields.letter === null && n.fields.hump_mm === null &&
            n.fields.obst_dist_cm === null && n.fields.obst_width_cm === null &&
            n.fields.speed_left === -1 && s.ok && s.fields.speed_left === null,
            "replayed bit kept, NONE fields decode as no data");

        /* bad frames are reported, not decoded */
        check("S1-U35",
            !decodeFrame([]).ok &&
            !decodeFrame(fromHex("02 c8 43 9b")).ok &&            /* version 2 */
            !decodeFrame(fromHex("39 00 00")).ok &&               /* type 7    */
            !decodeFrame(fromHex("01 d2 04 53")).ok &&            /* too short */
            !decodeFrame(fromHex("09 c8 43 9b 00")).ok &&         /* too long  */
            decodeFrame(fromHex("02 c8 43 9b")).error.indexOf("version") >= 0,
            "wrong version, type or length rejected with a reason");

        /* command and map encoders */
        check("S1-U35",
            same(encodeCmd("PAUSE").bytes, [1]) &&
            same(encodeCmd("resume").bytes, [2]) &&
            same(encodeCmd("ABORT").bytes, [3]) && !encodeCmd("GO").ok &&
            toHex(encodeMap("K", "D", "A", "Z").bytes) === "6a 80 0c" &&
            toHex(encodeMap("a", "b", "c", "d").bytes) === "20 88 01" &&
            !encodeMap("K", "K", "A", "Z").ok && !encodeMap("K", "", "A", "Z").ok &&
            !encodeMap("K", "1", "A", "Z").ok && !encodeMap("K", "DD", "A", "Z").ok,
            "cmd bytes 1/2/3, map K/D/A/Z = 6a 80 0c, bad maps refused");

        print("[TEST] dashboard SUMMARY " + pass + " passed, " + fail + " failed");
        return fail === 0;
    }

    var api = {
        LAYOUT_VERSION: LAYOUT_VERSION,
        TELEM_FIELDS: TELEM_FIELDS,
        MOVE_NAMES: MOVE_NAMES,
        SIDE_NAMES: SIDE_NAMES,
        decodeFrame: decodeFrame,
        encodeCmd: encodeCmd,
        encodeMap: encodeMap,
        letterName: letterName,
        stateName: stateName,
        toHex: toHex,
        fromHex: fromHex,
        selfTest: selfTest
    };

    if (typeof module !== "undefined" && module.exports) {
        module.exports = api;
        if (require.main === module) {
            process.exitCode = selfTest(console.log) ? 0 : 1;
        }
    } else {
        root.S1Codec = api;
    }
}(this));
