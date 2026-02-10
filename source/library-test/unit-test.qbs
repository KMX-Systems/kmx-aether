import qbs

CppApplication {
    Depends
    {
        name: 'kmx-aether-lib'
    }

    name: "kmx-aether-test"
    consoleApplication: true
    cpp.cxxLanguageVersion: "c++20"
    cpp.debugInformation: true
    cpp.enableRtti: false
    cpp.includePaths: [
        "inc",
        "inc_dep"
    ]
    cpp.staticLibraries: [
        "Catch2Main",
        "Catch2"
    ]
    files: [
        "inc/kmx/aether/**.hpp",
        "src/kmx/aether/**.cpp",
    ]
}
