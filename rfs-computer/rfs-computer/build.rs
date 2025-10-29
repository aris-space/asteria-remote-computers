//! Include build information

use std::io::Write as _;
use std::{env, fs, path, time};

fn main() {
    // Add git and cargo info
    built::write_built_file().expect("Failed to acquire build-time information");

    let dst = path::Path::new(&env::var("OUT_DIR").expect("OUT_DIR not set")).join("built.rs");

    let mut file = fs::OpenOptions::new()
        .append(true)
        .open(dst)
        .expect("must be able to write to build info file");

    writeln!(file, "// BELOW ARE CUSTOM ADDITIONS").expect("must be able to write");

    // Add BUILT_TIME_UNIX
    writeln!(
        file,
        "/// The time of build as a unix timestamp in seconds. `u32` lasts until 2038."
    )
    .expect("must be able to write");
    writeln!(file, "#[allow(dead_code)]").expect("must be able to write");
    writeln!(
        file,
        "pub static BUILT_TIME_UNIX: u32 = {};",
        time::SystemTime::now()
            .duration_since(time::SystemTime::UNIX_EPOCH)
            .expect("Time moved backwards")
            .as_secs()
    )
    .expect("must be able to write");

    // Add BUILDER_INITIALS
    writeln!(
        file,
        r"/// Initials of the person who compiled the firmware, or `\0\0` if not set. Determined using the BUILDER_INITIALS environment variable."
    )
    .expect("must be able to write");
    writeln!(file, "#[allow(dead_code)]").expect("must be able to write");
    writeln!(
        file,
        "pub static BUILDER_INITIALS: [u8; 2] = {:?};",
        <[u8; 2]>::try_from(
            env::var("BUILDER_INITIALS")
                .as_deref()
                .unwrap_or("\0\0")
                .as_bytes()
        )
        .expect("BUILDER_INITIALS must contain exactly two letters if set")
    )
    .expect("must be able to write");
}
