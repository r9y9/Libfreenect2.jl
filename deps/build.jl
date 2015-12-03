using BinDeps
using Compat

@BinDeps.setup

libfreenect2_version = "master"
libfreenect2 = library_dependency("libfreenect2")

github_root = "https://github.com/OpenKinect/libfreenect2"

provides(Sources,
         URI("$(github_root)/archive/$(libfreenect2_version).tar.gz"),
         libfreenect2,
         unpacked_dir="libfreenect2-$(libfreenect2_version)")

prefix = joinpath(BinDeps.depsdir(libfreenect2), "usr")
srcdir = joinpath(BinDeps.depsdir(libfreenect2), "src", "libfreenect2-$(libfreenect2_version)")

cmake_options = [
    "-DCMAKE_INSTALL_PREFIX=$prefix",
    "-DENABLE_CXX11=ON",
    "-DENABLE_OPENCL=ON",
    "-DENABLE_OPENGL=ON",
]

provides(SimpleBuild,
          (@build_steps begin
              GetSources(libfreenect2)
              @build_steps begin
                  ChangeDirectory(srcdir)
                  `mkdir -p build`
                  @build_steps begin
                      ChangeDirectory(joinpath(srcdir, "build"))
                      `rm -f CMakeCache.txt`
                      `cmake $cmake_options ..`
                      `make -j4`
                      `make install`
                  end
                end
          end), libfreenect2, os = :Unix)

@BinDeps.install @compat Dict(:libfreenect2 => :libfreenect2)
