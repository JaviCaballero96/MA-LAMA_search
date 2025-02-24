#include "external_function.h"

// #include "utils/system.h"

#include <dlfcn.h>
#include <iostream>

using namespace std;


ExternalFunctionManager::~ExternalFunctionManager() {
    for (void *handle : open_handles) {
        dlclose(handle);
    }
}

void ExternalFunctionManager::clear() {
    functions.clear();
    open_handles.clear();
}


void ExternalFunctionManager::load_module(const ExternalFunctionModuleInfo &module_info) {
    //std::string path = getenv("EF_PATH") + std::string("/");
    //std::string module_folder = (path==""?"./external_functions/":path);
    std::string module_folder = getenv("EF_PATH") == nullptr ? "./external_functions" : getenv("EF_PATH");
    module_folder += std::string("/");
    std::cout<<"Ef path: " << module_folder << '\n';
    cout << module_info.module_name << endl;
    std::string module_filename = module_folder + "lib" + module_info.module_name + ".so";
    std::cout << module_filename << std::endl;
    void *handle = dlopen(module_filename.c_str(), RTLD_LAZY);
    if (!handle) {
        cerr << "Cannot open library: " << dlerror() << endl;
        // utils::exit_with(utils::ExitCode::CRITICAL_ERROR);
        exit(1);
    }
    open_handles.push_back(handle);
    //Reset errors
    dlerror();

    SetupFunction setup_function = (SetupFunction) dlsym(handle, "setup");
    const char *dlsym_setup_error = dlerror();

    if (dlsym_setup_error != NULL) {
        cerr << "Cannot execute function setup function in " << module_filename << endl;
    }
    else {
        setup_function();
    }

    for (const ExternalFunctionInfo &function_info : module_info.functions) {
        ExternalFunction function = (ExternalFunction) dlsym(handle, function_info.name.c_str());
        const char *dlsym_error = dlerror();
        if (dlsym_error != NULL) {
            cerr << "Cannot find function '" << function_info.name << "' in "
                 << module_filename << ": " << dlsym_error << endl;
            // utils::exit_with(utils::ExitCode::CRITICAL_ERROR);
            exit(1);
        }

        for (const GroundedExternalFunctionInfo &instance_info : function_info.instances) {
            GroundedExternalFunction instance = make_pair(function, instance_info.parameters);
            if (functions.count(instance_info.var) > 0) {
                cerr << "Duplicate external function instance for variable: " << instance_info.var << endl;
                // utils::exit_with(utils::ExitCode::INPUT_ERROR);
                exit(1);
            }
            functions[instance_info.var] = instance;
            std::cout << "external function #" << instance_info.var << ", " << instance_info.name << ": " << compute_function(instance_info.var) << endl;
        }
    }
}

double ExternalFunctionManager::compute_function(int variable_id) {
    if (functions.count(variable_id) == 0) {
        cerr << "No external function loaded for variable: " << variable_id << endl;
        // utils::exit_with(utils::ExitCode::CRITICAL_ERROR);
        exit(1);
    }
    const GroundedExternalFunction &instance = functions[variable_id];
    float res = instance.first(instance.second);
    // cout << "Res computed: " << res << endl;
    return res;
}
