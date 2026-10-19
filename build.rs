use std::process::Command;

fn main() {
    // Capture the short commit hash at build time so it can be embedded in the
    // startup header. Fall back to "unknown" when building outside a git
    // working tree (e.g. from a source tarball).
    let hash = Command::new("git")
        .args(["rev-parse", "--short", "HEAD"])
        .output()
        .ok()
        .filter(|output| output.status.success())
        .map(|output| String::from_utf8_lossy(&output.stdout).trim().to_owned())
        .unwrap_or_else(|| "unknown".to_owned());

    println!("cargo:rustc-env=GIT_COMMIT_HASH={hash}");
}
