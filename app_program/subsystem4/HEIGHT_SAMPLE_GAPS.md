# Powered hump sampling

The IMU task uses explicit quality metadata. A short no-new-sample or non-gravity-dominant interval is deferred, never used as a valid angle. Only qualified endpoint angles and real cumulative forward wheel distance are integrated. The endpoint interval must satisfy the existing 60 ms / 50 mm limits. At a 20 ms task period this permits at most two intervening missed readings; it does not increase the 40 ms odometry age limit. I/O errors, saturation, impacts, clock faults, reversing, missing baseline and longer gaps still invalidate the traversal. The original strict API remains available.

`interpolated` and `max_interp_us` identify this approximation. These development guards and physical height error still require measured validation; host checks are not proof of ±10% accuracy. No measurement ranges or packed message offsets are changed. Sampling, sensor gravity checks, GPIO ownership, Safety Monitor fault handling and magnetic settings remain unchanged.

The live console reports `encoder_valid` separately from `odom_valid`: the latter indicates a usable angle/distance pair and can be false while the encoder reader is healthy. `imu_status`, `dynamic_mg`, `pending_imu` and `chain_valid` expose the first rejection. They are diagnostic serial fields, not a confirmed S4/S5 packed interface.

For the local minimal line/height demo only, the build helper adds a bounded forward traversal policy while a height candidate is active. The real line task continues to run and its unmodified observations remain visible. Line search is suspended during that traversal and resumes afterward. Safety and supervisor stops retain precedence. This demo override is outside the committed team controller and is not a production interface agreement.
