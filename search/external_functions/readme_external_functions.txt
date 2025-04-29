## Manual for Creating an External Function for MA-LAMA

This manual provides instructions on how to create an external function for MA-LAMA. An external function consists of two components: a shared library that defines various functions and a PDDL description that specifies the function within the PDDL domain.

### Creating the Library

The external function library must be shared and can include two types of functions that the planner can use:

1. **Configuration Function (Optional)**
   - This function is executed when the external function is loaded at the start of a planning episode.
   - It does not take any parameters.

   ```cpp
   void setup() {
       // Insert your code here
   }
   ```

2. **Action Functions**
   - These functions represent system features such as path planning costs or energy costs between waypoints.
   - Each function takes a vector of strings as parameters, corresponding to the PDDL representation.
   - They must always return a double value.

   ```cpp
   double function_name(std::vector<std::string> parameters) {
       // Insert your code here
   }
   ```

All libraries must be stored in the `external_function` folder inside the MA-LAMA planner.

### Writing the PDDL Description

The PDDL description must be included in the Domain file. If it is not present, the functions will not be loaded or executed at the start of each planning episode. The PDDL syntax for defining external functions follows this structure:

```pddl
(:modules
  (:module module_name_1
    (:function (function_name_1 ?p1 - parameter_type ?p2 - parameter_type))
    (:function (function_name_2 ?p3 - parameter_type))
  )
  (:module module_name_2
    (:function (function_name_3 ?p2 - parameter_type))
  )
)
```

### Example Explanation

The example above defines two external libraries:
- **Module `module_name_1`** provides two functions.
- **Module `module_name_2`** provides one function.

The function syntax resembles a predicate, and the module name must match the corresponding library. During the loading process, the system constructs the library name based on the module name in the following manner:

```
function_name_1  -->  libfunction_name_1.so
```

The library must be named according to the module name used in PDDL.
