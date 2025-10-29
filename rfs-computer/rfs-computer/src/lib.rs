//! All IO types, config types and state logic blocks are in the library.
//!
//! The binary then uses these types to contructs the full program.

pub mod config;
pub mod io;
pub mod state_logic;

pub mod build_info {
    use lib_mission::util::init::BuildInfo;

    /// Collects the important information from this build.
    pub const THIS_BUILD: BuildInfo = BuildInfo {
        // This crate has a `main.rs` thus using the package is as expected.
        binary: built_info::PKG_NAME,
        git_commit: unwrap_str(built_info::GIT_COMMIT_HASH),
        git_tag: unwrap_str(built_info::GIT_VERSION),
        git_dirty: if let Some(d) = built_info::GIT_DIRTY {
            d
        } else {
            true
        },
        is_release: !built_info::DEBUG, // Using [`built_info::PROFILE`] is const incompatible
        features: built_info::FEATURES_STR,
        unix_timestamp: built_info::BUILT_TIME_UNIX,
        author_initials: built_info::BUILDER_INITIALS,
    };

    #[allow(unknown_lints)]
    #[allow(clippy::pedantic)]
    mod built_info {
        include!(concat!(env!("OUT_DIR"), "/built.rs"));
    }

    // `expect` is only stable in 1.83, and unwrap_or not even now in const contexts.
    const fn unwrap_str(o: Option<&'static str>) -> &'static str {
        if let Some(s) = o { s } else { "{unknown}" }
    }
}
