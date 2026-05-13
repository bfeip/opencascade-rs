/// Minimum compatible version of OpenCASCADE library (major, minor)
///
/// Pre-installed OpenCASCADE library will be checked for compatibility using semver rules.
const OCCT_VERSION: (u8, u8) = (7, 8);

/// The list of used OpenCASCADE libraries which needs to be linked with.
const OCCT_LIBS: &[&str] = &[
    "TKMath",
    "TKernel",
    "TKDE",
    "TKFeat",
    "TKGeomBase",
    "TKG2d",
    "TKG3d",
    "TKTopAlgo",
    "TKGeomAlgo",
    "TKBRep",
    "TKPrim",
    "TKDESTEP",
    "TKDEIGES",
    "TKDESTL",
    "TKMesh",
    "TKShHealing",
    "TKFillet",
    "TKBool",
    "TKBO",
    "TKOffset",
    "TKXSBase",
    "TKCAF",
    "TKCDF",
    "TKHLR",
    "TKLCAF",
    "TKXCAF",
    "TKVCAF",
    "TKService",
    "TKV3d",
];

fn main() {
    let target = std::env::var("TARGET").expect("No TARGET environment variable defined");
    let is_windows = target.to_lowercase().contains("windows");
    let is_windows_gnu = target.to_lowercase().contains("windows-gnu");
    let is_emscripten = target.contains("emscripten");

    let occt_config = OcctConfig::detect(&target);

    println!("cargo:rustc-link-search=native={}", occt_config.library_dir.to_str().unwrap());

    let lib_type = if occt_config.is_dynamic { "dylib" } else { "static" };
    for lib in OCCT_LIBS {
        println!("cargo:rustc-link-lib={lib_type}={lib}");
    }

    if is_windows {
        println!("cargo:rustc-link-lib=dylib=user32");
    }

    let mut build = cxx_build::bridge("src/lib.rs");

    if is_windows_gnu {
        build.define("OCC_CONVERT_SIGNALS", "TRUE");
    }

    if let "windows" = std::env::consts::OS {
        let current = std::env::current_dir().unwrap();
        build.include(current.parent().unwrap());
    }

    if is_emscripten {
        // cc-rs adds -fno-exceptions for Emscripten by default. -fexceptions re-enables them,
        // then -fwasm-exceptions switches to WASM native exception mode. Both flags are needed
        // because -fwasm-exceptions alone does not override a preceding -fno-exceptions.
        build.flag("-fexceptions").flag("-fwasm-exceptions");
    }

    build
        .cpp(true)
        .flag_if_supported("-std=c++11")
        .define("_USE_MATH_DEFINES", "TRUE")
        .include(occt_config.include_dir)
        .include("include")
        .compile("wrapper");

    println!("cargo:rustc-link-lib=static=wrapper");

    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=include/wrapper.hxx");
}

struct OcctConfig {
    include_dir: std::path::PathBuf,
    library_dir: std::path::PathBuf,
    is_dynamic: bool,
}

impl OcctConfig {
    fn detect(target: &str) -> Self {
        if target.contains("emscripten") {
            Self::for_emscripten()
        } else {
            Self::for_native()
        }
    }

    /// Locate OCCT for `wasm32-unknown-emscripten`.
    ///
    /// Resolution order:
    ///
    /// 1. `OCCT_EMSCRIPTEN_ROOT` — path to a pre-built Emscripten OCCT install. Use this to
    ///    skip a long cmake build (e.g. in CI with a cached artifact).
    ///
    /// 2. `builtin` feature — builds OCCT from source via the Emscripten cmake toolchain,
    ///    found at `$EMSDK/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake`.
    ///    Requires `EMSDK` to be set in the environment.
    fn for_emscripten() -> Self {
        println!("cargo:rerun-if-env-changed=OCCT_EMSCRIPTEN_ROOT");

        if let Ok(root) = std::env::var("OCCT_EMSCRIPTEN_ROOT") {
            return Self::from_install_root(std::path::PathBuf::from(root), false);
        }

        #[cfg(feature = "builtin")]
        {
            let emsdk = std::env::var("EMSDK").expect(
                "EMSDK must be set to use the `builtin` feature when targeting wasm32-unknown-emscripten"
            );
            let toolchain = std::path::PathBuf::from(&emsdk)
                .join("upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake");
            if !toolchain.exists() {
                panic!(
                    "Emscripten cmake toolchain not found at {}.\n\
                     Ensure EMSDK points to a valid emsdk installation (run emsdk_env.sh first).",
                    toolchain.display()
                );
            }
            occt_sys::build_occt(Some(&toolchain));
            return Self::from_install_root(occt_sys::occt_emscripten_path(), false);
        }

        #[cfg(not(feature = "builtin"))]
        panic!(
            "OCCT_EMSCRIPTEN_ROOT must be set when targeting wasm32-unknown-emscripten.\n\
             Build OCCT with Emscripten (emcmake cmake ... && emmake make install) and set\n\
             OCCT_EMSCRIPTEN_ROOT to the install prefix.\n\
             Alternatively, enable the `builtin` feature with EMSDK set in the environment."
        );
    }

    /// Locate a native OCCT installation via cmake's `find_package(OpenCASCADE)`.
    ///
    /// Resolution order:
    ///
    /// 1. `builtin` feature — builds OCCT from source inside cargo.
    ///
    /// 2. `DEP_OCCT_ROOT` — cmake prefix-path hint pointing to a pre-installed OCCT.
    ///
    /// 3. System-wide OCCT discovered by cmake automatically.
    fn for_native() -> Self {
        println!("cargo:rerun-if-env-changed=DEP_OCCT_ROOT");

        #[cfg(feature = "builtin")]
        {
            occt_sys::build_occt(None);
            std::env::set_var("DEP_OCCT_ROOT", occt_sys::occt_path().as_os_str());
        }

        let dst =
            std::panic::catch_unwind(|| cmake::Config::new("OCCT").register_dep("occt").build());

        #[cfg(feature = "builtin")]
        let dst = dst.expect("Builtin OpenCASCADE library not found.");

        #[cfg(not(feature = "builtin"))]
        let dst = dst.expect("Pre-installed OpenCASCADE library not found. You can use `builtin` feature if you do not want to install OCCT libraries system-wide.");

        let cfg = std::fs::read_to_string(dst.join("share").join("occ_info.txt"))
            .expect("Something went wrong when detecting OpenCASCADE library.");

        let mut version_major: Option<u8> = None;
        let mut version_minor: Option<u8> = None;
        let mut include_dir: Option<std::path::PathBuf> = None;
        let mut library_dir: Option<std::path::PathBuf> = None;
        let mut is_dynamic: bool = false;

        for line in cfg.lines() {
            if let Some((var, val)) = line.split_once('=') {
                match var {
                    "VERSION_MAJOR" => version_major = val.parse().ok(),
                    "VERSION_MINOR" => version_minor = val.parse().ok(),
                    "INCLUDE_DIR" => include_dir = val.parse().ok(),
                    "LIBRARY_DIR" => library_dir = val.parse().ok(),
                    "BUILD_SHARED_LIBS" => is_dynamic = val == "ON",
                    _ => (),
                }
            }
        }

        if let (Some(version_major), Some(version_minor), Some(include_dir), Some(library_dir)) =
            (version_major, version_minor, include_dir, library_dir)
        {
            if version_major != OCCT_VERSION.0 || version_minor < OCCT_VERSION.1 {
                #[cfg(feature = "builtin")]
                panic!("Builtin OpenCASCADE library found but version is not met (found {}.{} but {}.{} required). Please fix OCCT_VERSION in build script of `opencascade-sys` crate or submodule OCCT in `occt-sys` crate.",
                       version_major, version_minor, OCCT_VERSION.0, OCCT_VERSION.1);

                #[cfg(not(feature = "builtin"))]
                panic!("Pre-installed OpenCASCADE library found but version is not met (found {}.{} but {}.{} required). Please provide required version or use `builtin` feature.",
                       version_major, version_minor, OCCT_VERSION.0, OCCT_VERSION.1);
            }

            Self { include_dir, library_dir, is_dynamic }
        } else {
            panic!("OpenCASCADE library found but something wrong with config.");
        }
    }

    /// Construct an `OcctConfig` from a known install root.
    ///
    /// Handles both flat (`include/`) and nested (`include/opencascade/`) header layouts.
    fn from_install_root(root: std::path::PathBuf, is_dynamic: bool) -> Self {
        let include_dir = {
            let nested = root.join("include").join("opencascade");
            if nested.exists() { nested } else { root.join("include") }
        };
        Self { include_dir, library_dir: root.join("lib"), is_dynamic }
    }
}
