module TaskTerminate
    export close_task

    """
        close_task(tasks::Dict{String, Task}; exclude::Union{Nothing,Tuple} = nothing)
    
    This function handle the termination of an open task in julia,
    the inputs to the function is a Tasks dictionry which as keys has the task name
    and the values are the pointers to the tasks. The `exclude` input is optional 
    and once given the tasks names with in this tuple will not be terminated.
    ```@example
        taskname = "dumy_task"
        tasks = Dict{String,Task}()
        tasks[taskname] = @async begin
            while true
                println("Hello, This dumy_task is Runing")
                sleep(1)
            end
        end
        sleep(4)
        close_task(tasks)
    ```
    """
    function close_task(tasks::Dict{String, Task}; exclude::Union{Nothing,Tuple} = nothing)
        h = @async begin
            printstyled("starting to close taskes\n", color = :yellow)
            if !isnothing(exclude)
                tasks_to_kill = deepcopy(tasks)
                for ex in exclude
                    if haskey(tasks_to_kill, ex)
                        delete!(tasks_to_kill, ex)
                    end
                end
            else
                tasks_to_kill = tasks
            end

            if isempty(tasks_to_kill)
                printstyled("finish clossing all the needed tasks\n", color = :green)
                Base.throwto(h, InterruptException())
            end

            for (key,val) in tasks_to_kill
                while !istaskdone(val)
                    @async close_task(key, val)
                    sleep(0.5)
                end
                printstyled("finish closing task $key\n", color = :green)
                delete!(tasks, key)
            end

            sleep(0.1)
            println("============================================================")
            printstyled("finish clossing all the needed tasks\n", color = :green)
            Base.throwto(h, InterruptException())
        end
    end

    function close_task(disc::String, task::Task)
        try
            printstyled("clossing Task $disc\n", color = :red)
            Base.throwto(task, InterruptException())
        catch
            return nothing
        end
    end    
end # module
