//! 构建脚本：挑选本仓库携带的原生库、生成链接参数、把运行期 DLL 投放到 cargo 的输出目录。
//!
//! 原生库来源（优先级从高到低）：
//!   1. 环境变量 `DMCAN_SDK_DIR`：可指向 C++ SDK 交付包的**平台目录**（内含 `sdk/`），
//!      也可直接指向那个 `sdk/` 目录，或本仓库 `native/<平台>` 目录；
//!   2. 本仓库 `native/<平台>/`（随仓库携带的预编译交付件）。

use std::env;
use std::fs;
use std::path::{Path, PathBuf};

/// 本仓库 `native/` 下的平台目录名，与 C++ SDK 交付包的目录名保持一致。
fn platform_dir() -> Option<&'static str> {
    let os = env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    let arch = env::var("CARGO_CFG_TARGET_ARCH").unwrap_or_default();
    let abi = env::var("CARGO_CFG_TARGET_ENV").unwrap_or_default();
    Some(match (os.as_str(), arch.as_str()) {
        // Windows 上 MSVC 与 MinGW 两套交付包的库文件名不同，目录也分开放
        ("windows", "x86_64") | ("windows", "aarch64") if abi == "gnu" => "mingw",
        ("windows", _) => "msvc",
        ("linux", "x86_64") => "linux_x86_64",
        ("linux", "aarch64") => "linux_aarch64",
        ("macos", "aarch64") => "macos_arm64",
        ("macos", "x86_64") => "macos_x86_64",
        _ => return None,
    })
}

/// 把 `DMCAN_SDK_DIR` 归一化成"含 `include/` 与 `lib/`（或 `bin/`）的目录"。
fn normalize_sdk_dir(dir: PathBuf) -> PathBuf {
    if dir.join("sdk").is_dir() {
        dir.join("sdk")
    } else {
        dir
    }
}

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=DMCAN_SDK_DIR");

    // docs.rs 在 Linux 上只生成文档、且包里不带任何原生库：跳过链接与 DLL 投放，
    // 否则会在这里 panic，crate 页面就会显示"文档构建失败"（docs.rs 会设 DOCS_RS=1）。
    if env::var_os("DOCS_RS").is_some() {
        println!("cargo:warning=DOCS_RS 已设置：跳过原生库链接，仅生成文档");
        return;
    }

    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR"));
    let platform = platform_dir();
    let sdk_dir = match env::var_os("DMCAN_SDK_DIR") {
        Some(dir) => normalize_sdk_dir(PathBuf::from(dir)),
        None => {
            let name = platform.unwrap_or_else(|| {
                panic!(
                    "dmcan: 本仓库暂不支持当前目标平台。请设置环境变量 DMCAN_SDK_DIR \
                     指向 C++ SDK 交付包的平台目录（例如 <包>/linux_x86_64 或它下面的 sdk/）。"
                )
            });
            manifest_dir.join("native").join(name)
        }
    };

    if !sdk_dir.is_dir() {
        panic!(
            "dmcan: 找不到原生库目录 {}。两种修法：① 在仓库里补上 native/<平台>/ 交付件；\
             ② 设置环境变量 DMCAN_SDK_DIR 指向 C++ SDK 交付包的平台目录。",
            sdk_dir.display()
        );
    }
    println!("cargo:rerun-if-changed={}", sdk_dir.display());

    // 链接：头文件不需要参与编译（绑定是手写的），只需要库目录与库名。
    let lib_dir = if sdk_dir.join("lib").is_dir() {
        sdk_dir.join("lib")
    } else {
        sdk_dir.clone()
    };
    println!("cargo:rustc-link-search=native={}", lib_dir.display());
    println!("cargo:rustc-link-lib=dylib=dmcan");

    // 运行期库投放：把交付目录里的动态库拷进 cargo 的运行目录，让 `cargo run` / `cargo test` /
    // 示例都不用额外配置。三个平台各有各的理由，但结果一样——库必须落在可执行文件旁边：
    //   - Windows：没有 rpath，Loader 只在 exe 同目录（或 PATH）找 `*.dll`；
    //   - Linux：`libdmcan.so` 的 SONAME 是 `libdmcan.so`，靠我们在下面 emit 的 `$ORIGIN` 找；
    //   - macOS：`libdmcan.dylib` 的 install name 是 `@rpath/libdmcan.dylib`，靠 `@loader_path` 找。
    // 注意 macOS：`libdmcan.dylib` 内部按 `@loader_path/libusb-1.0.0.dylib` 找 libusb，
    // 所以 libusb 必须与**它**同目录（我们按"不随包分发"不带它，由使用方自备）。
    let target_os = env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    let runtime_dir = if target_os == "windows" {
        sdk_dir.join("bin")
    } else {
        lib_dir.clone()
    };
    let out_dir = PathBuf::from(env::var("OUT_DIR").expect("OUT_DIR"));
    // OUT_DIR = target/<profile>/build/<pkg>-<hash>/out，上溯三级就是 target/<profile>。
    let profile_dir = out_dir
        .ancestors()
        .nth(3)
        .expect("target/<profile> 目录")
        .to_path_buf();
    for dst in [profile_dir.clone(), profile_dir.join("deps")] {
        copy_runtime_libs(&runtime_dir, &dst, &target_os);
    }

    match target_os.as_str() {
        "windows" => {
            // crates.io 包里不带 libusb-1.0.dll（LGPL-2.1，由使用方自行提供）：缺了它 dmcan.dll
            // 在运行期加载不起来，这里给一条明确提示，别让用户自己去猜。
            if !runtime_dir.join("libusb-1.0.dll").is_file() {
                println!(
                    "cargo:warning=未在 {} 找到 libusb-1.0.dll：它不在 crates.io 包里（LGPL-2.1），\
                     请自备一份（放进 PATH 或可执行文件同目录），或设 DMCAN_SDK_DIR 指向完整的 SDK 交付包",
                    runtime_dir.display()
                );
            }
        }
        "macos" => {
            println!("cargo:rustc-link-arg=-Wl,-rpath,@loader_path");
            // 同上：libusb 不在包里，但 `libdmcan.dylib` 硬依赖 `@loader_path/libusb-1.0.0.dylib`。
            if !runtime_dir.join("libusb-1.0.0.dylib").is_file() {
                println!(
                    "cargo:warning=未在 {} 找到 libusb-1.0.0.dylib：`libdmcan.dylib` 按 \
                     `@loader_path/libusb-1.0.0.dylib` 找它，必须与 libdmcan.dylib 同目录。\
                     它不在 crates.io 包里（LGPL-2.1），请自备（或设 DMCAN_SDK_DIR 指向完整的 SDK 交付包）。",
                    runtime_dir.display()
                );
            }
        }
        _ => {
            // Linux：libusb 由系统提供（如 `apt install libusb-1.0-0`），这里只设 rpath。
            println!("cargo:rustc-link-arg=-Wl,-rpath,$ORIGIN");
        }
    }
}

/// 判断某个文件名是否是该平台的"运行期动态库"。
fn is_runtime_lib(name: &str, target_os: &str) -> bool {
    match target_os {
        "windows" => name.ends_with(".dll"),
        "macos" => name.contains(".dylib"),
        // Linux 的 `libdmcan.so` / `libdmcan.so.2` / `.so.2.1.0` 都要能命中
        _ => name.contains(".so"),
    }
}

/// 把 `src` 目录下该平台的运行期动态库拷到 `dst`（含它们的运行期依赖，例如 libusb）。
fn copy_runtime_libs(src: &Path, dst: &Path, target_os: &str) {
    let entries = match fs::read_dir(src) {
        Ok(entries) => entries,
        Err(_) => return,
    };
    if fs::create_dir_all(dst).is_err() {
        return;
    }
    let _ = entries.flatten().try_for_each(|entry| {
        let path = entry.path();
        let name = entry.file_name().to_string_lossy().into_owned();
        if !is_runtime_lib(&name, target_os) {
            return Ok(());
        }
        let target = dst.join(entry.file_name());
        fs::copy(&path, &target).map(|_| ())
    });
}
