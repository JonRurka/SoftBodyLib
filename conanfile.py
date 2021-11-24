from conans import ConanFile

class SoftBodyLib(ConanFile):
    name = "SoftBodyLib"
    version = "1.0"
    license = "Properitary"
    description = "GPU accelorated soft body physics"

    requires = (
        "spdlog/1.8.0",
        "boost/1.75.0",
        "glm/0.9.9.8"
    )

    settings = "os", "compiler", "build_type", "arch"
    generators = "cmake"
    exports_sources = "*", "!*build/*"

    def build(self):
        cmake.configure()
        cmake.build()

    def imports(self):
        self.copy('*.so*', dst='lib', src='lib')
