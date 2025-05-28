using Libdl

# Load library
lib = Libdl.dlopen("./build/libProject_user.so")

# Get function symbols
init_func       = Libdl.dlsym(lib, :init)
free_func       = Libdl.dlsym(lib, :free_resources)
philippides_func = Libdl.dlsym(lib, :philippides)
get_res_func    = Libdl.dlsym(lib, :get_philippides_results)

# Now call with function pointers

function call_init()
    ccall(init_func, Cvoid, ())
end

function call_philippides(x::Vector{Float64})
    ccall(philippides_func, Cvoid, (Ptr{Float64},), x)
end

function call_free_resources()
    ccall(free_func, Cvoid, ())
end

function get_results()
    res = Vector{Float64}(undef, 8)
    ccall(get_res_func, Cvoid, (Ptr{Float64},), res)
    return res
end


# Run the sequence of function calls
function run_simulation()
    # Step 1: Initialize the MBS model and data
    println("Initializing MBS model...")
    cd("build") do
        call_init()
        # Step 2: Call philippides 5 times with different input values and retrieve results
        for i in 1:3
            # Generate a different set of input values for each call
            #x = zeros(16)  # Example: generate a vector of 16 random doubles
            x = [0.0, 0.0, 1.0, -1.0, 0.0, 1.0, -1.0, 0.0,    # q
                 0.0, 0.0, -0.1, 0.1, 0.0, -0.1, 0.1, 0.0,    # qd
                 2.4, 2.4, -0.6, 0.0]                         # U_REF
            println("Running philippides with input #$i: ", x)
            call_philippides(x)
            
            # Retrieve the results after the call to philippides()
            results = get_results()
            println("Results from philippides call #$i: ", results)
        end

        # Step 3: Free allocated resources
        println("Freeing resources...")
        call_free_resources()
    end
end

# Run the simulation
run_simulation()
