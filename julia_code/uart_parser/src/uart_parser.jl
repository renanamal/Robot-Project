using LibSerialPort, DelimitedFiles

include(joinpath(@__DIR__,"../SerialMsgs/src/SerialMsgs.jl"))
using .SerialMsgs

include(joinpath(@__DIR__,"../TaskTerminate/src/TaskTerminate.jl"))
using .TaskTerminate

mutable struct MsgIn
    data::Float32
end
N =  length(fieldnames(MsgIn))
MsgIn() = MsgIn(zeros(N)...)

const DATA_SIZE = sizeof(MsgIn) # num of bytes
const MSG_LEN = DATA_SIZE + 2 # 1 byte header and 1 byte footer

const HEADER = 0x01
const FOOTER = 0x05

tasks = Dict{String,Task}()


list_ports()
# Modify these as needed
portname = "COM14"
baudrate = 115200

f = open(joinpath(@__DIR__,"../records/metry.txt"), "w")

input_buff = Vector{UInt8}()
sp = LibSerialPort.open(portname, baudrate)
msgin = MsgIn()

try
    for i in 1:10

        while bytesavailable(sp) ≤ sizeof(MSG_LEN)
        end

        while bytesavailable(sp) > 0
            tmp = read(sp)
            append!(input_buff,tmp)
        end
        start_ind = 1
        current_length = length(input_buff)
        while MSG_LEN ≤ current_length
            end_ind = start_ind + MSG_LEN - 1
            if input_buff[start_ind] == HEADER && input_buff[end_ind] == FOOTER
                SerialMsgs.serial2struct!(msgin, input_buff[start_ind+1:end_ind-1])
                writedlm(f, msgin.data', ',')
                deleteat!(input_buff,1:end_ind)
                current_length = length(input_buff)
                @info current_length
                start_ind = 1
            else
                start_ind += 1
                if start_ind + MSG_LEN - 1 > current_length
                    @info "Msg not found clear buffer"
                    @show input_buff
                    input_buff = Vector{UInt8}()
                    current_length = 0
                    start_ind = 1
                end
            end
        end
    end
catch e
    @show e
    close(f)
    close(sp)
finally
    close(f)
    close(sp)
end

