import qbs

StaticLibrary {
    Depends { name: "cpp" }
    consoleApplication: true
    cpp.cxxLanguageVersion: "c++26"
    cpp.enableRtti: false
    cpp.includePaths: [
        "inc",
        "inc_dep",
    ]
    install: true
    name: "kmx-aether-lib"
    files: [
        "inc/kmx/**.hpp",
        "inc/kmx/aether/**.hpp",
    ]
}
