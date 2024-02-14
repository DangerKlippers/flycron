fn main() {
    anchor_codegen::ConfigBuilder::new()
        .entry("src/main.rs")
        .set_version(format!("Flying Gantry {}", env!("CARGO_PKG_VERSION")))
        .set_build_versions("")
        .build();
}
