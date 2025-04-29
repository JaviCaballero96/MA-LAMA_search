#ifndef EXTERNAL_FUNCTIONS_H
#define EXTERNAL_FUNCTIONS_H

#include <string>
#include <unordered_map>
#include <vector>

//#include <bits/stdc++.h>

struct GroundedExternalFunctionInfo {
    int var;
    std::string name;
    std::vector<std::string> parameters;
};

struct ExternalFunctionInfo {
    std::string name;
    std::vector<GroundedExternalFunctionInfo> instances;
};

struct ExternalFunctionModuleInfo {
    std::string module_name;
    std::vector<ExternalFunctionInfo> functions;
};

class ExternalFunctionManager {
    typedef double (*ExternalFunction)(const std::vector<std::string> &);
    typedef void (*SetupFunction)();
    using GroundedExternalFunction = std::pair<ExternalFunction, std::vector<std::string>>;
    std::vector<void*> open_handles;

    std::unordered_map<int, GroundedExternalFunction> functions;
public:
    ~ExternalFunctionManager();

    void load_module(const ExternalFunctionModuleInfo &module_info);

    double compute_function(int variable_id);
    void clear();
};


#endif
