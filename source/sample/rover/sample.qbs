import qbs

CppApplication {
    Depends { name: "kmx-aether-lib" }

    name: "rover-sample"
    consoleApplication: true
    cpp.cxxLanguageVersion: "c++26"
    cpp.debugInformation: true
    cpp.enableRtti: false
    cpp.includePaths: [
        "inc",
        "inc_dep"
    ]
    cpp.staticLibraries: [
    ]
    files: [
        "src/main.cpp",
    ]
}
