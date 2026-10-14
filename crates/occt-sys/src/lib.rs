use std::{
    env::var,
    path::{Path, PathBuf},
};

const LIB_DIR: &str = "lib";
const INCLUDE_DIR: &str = "include";

/// Get the path to the native OCCT library installation directory.
///
/// Only valid during build (`cargo clean` removes these files).
pub fn occt_path() -> PathBuf {
    // moves the output into target/TARGET/OCCT
    // this way its less likely to be rebuilt without a cargo clean
    Path::new(&var("OUT_DIR").expect("missing OUT_DIR")).join("../../../../OCCT")
}

/// Get the path to the Emscripten OCCT library installation directory.
///
/// Only valid during build (`cargo clean` removes these files).
pub fn occt_emscripten_path() -> PathBuf {
    Path::new(&var("OUT_DIR").expect("missing OUT_DIR")).join("../../../../OCCT-emscripten")
}

/// Build the OCCT library.
///
/// Pass `Some(toolchain)` pointing to Emscripten's cmake toolchain file to target
/// `wasm32-unknown-emscripten`; pass `None` for a native build. Output is placed in
/// `occt_emscripten_path()` or `occt_path()` respectively.
pub fn build_occt(emscripten_toolchain: Option<&Path>) {
    let out_dir = if emscripten_toolchain.is_some() {
        occt_emscripten_path()
    } else {
        occt_path()
    };

    let mut cfg = cmake::Config::new(Path::new(env!("OCCT_SRC_DIR")));
    cfg.define("BUILD_PATCH", Path::new(env!("OCCT_PATCH_DIR")))
        .define("BUILD_LIBRARY_TYPE", "Static")
        .define("BUILD_MODULE_ApplicationFramework", "FALSE")
        .define("BUILD_MODULE_Draw", "FALSE")
        .define("USE_D3D", "FALSE")
        .define("USE_DRACO", "FALSE")
        .define("USE_EIGEN", "FALSE")
        .define("USE_FFMPEG", "FALSE")
        .define("USE_FREEIMAGE", "FALSE")
        .define("USE_FREETYPE", "FALSE")
        .define("USE_GLES2", "FALSE")
        .define("USE_OPENGL", "FALSE")
        .define("USE_OPENVR", "FALSE")
        .define("USE_RAPIDJSON", "FALSE")
        .define("USE_TBB", "FALSE")
        .define("USE_TCL", "FALSE")
        .define("USE_TK", "FALSE")
        .define("USE_VTK", "FALSE")
        .define("USE_XLIB", "FALSE")
        .define("INSTALL_DIR_LIB", LIB_DIR)
        .define("INSTALL_DIR_INCLUDE", INCLUDE_DIR);

    if let Some(toolchain) = emscripten_toolchain {
        // OCCT defines OCC_CONVERT_SIGNALS on every non-MSVC platform, which puts a setjmp
        // inside each OCC_CATCH_SIGNALS try block. WebAssembly has no signals to convert, and
        // LLVM 23 miscompiles setjmp inside try under -fwasm-exceptions into an invalid
        // br_table (llvm/llvm-project#230956). The -U follows OCCT's -D on the command line.
        let flags = "-fwasm-exceptions -UOCC_CONVERT_SIGNALS";
        cfg.define("CMAKE_TOOLCHAIN_FILE", toolchain)
            .define("CMAKE_CXX_FLAGS", flags)
            .define("CMAKE_C_FLAGS", flags);
    }

    cfg.profile("Release").out_dir(out_dir).build();
}
