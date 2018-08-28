#
# Main function for Hybrid A* Trailer 
# 
# Author: Atsushi Sakai(@Atsushi_twi)
#
using PyPlot    
include("./lib/trailer_hybrid_a_star_test.jl")

function main()
    println(PROGRAM_FILE," start!!")

    # set obstacles
    ox = Float64[]
    oy = Float64[]

    for i in -25:25
        push!(ox, Float64(i))
        push!(oy, 47.0)
    end
    for i in -25:-3
        push!(ox, Float64(i))
        push!(oy, 3.0)
    end
    for i in -15:3
        push!(ox, -3.0)
        push!(oy, Float64(i))
    end
    for i in -15:3
        push!(ox, 3.0)
        push!(oy, Float64(i))
    end
    for i in 3:25
        push!(ox, Float64(i))
        push!(oy, 3.0)
    end
    for i in -3:3
        push!(ox, Float64(i))
        push!(oy, -15.0)
    end

    oox = ox[:]
    ooy = oy[:]

    # goal state
    gx = 0.0  # [m]
    gy = 0.0  # [m]
    gyaw0 = deg2rad(90.0)
    gyaw1 = deg2rad(90.0)

    # path generation
    h_dp, ox, oy, c, kdtree= trailer_hybrid_a_star.map_initial(gx, gy, gyaw0, gyaw1, ox, oy,trailer_hybrid_a_star.XY_GRID_RESOLUTION, trailer_hybrid_a_star.YAW_GRID_RESOLUTION)
    scenarios_allnum = 0
    scenarios_success = 0

    # all scenarios with different position and heading angles
    for sy in 15.0:2:35.0  # [m]
        for sx in -20.0:2:20.0 # [m]
            for angle0 in 0.0:15.0:360.0
                for difangle1 in -30.0:10.0:30.0
                    syaw0 = deg2rad(angle0)
                    syaw1 = deg2rad(angle0 + difangle1)
                    println("!!!!Start!!!!!")
                    path = trailer_hybrid_a_star.calc_hybrid_astar_path(sx, sy, syaw0, syaw1, gx, gy, gyaw0, gyaw1, ox, oy,trailer_hybrid_a_star.XY_GRID_RESOLUTION, trailer_hybrid_a_star.YAW_GRID_RESOLUTION, h_dp, c, kdtree)
                    if path != nothing
                        plot_path(path, oox, ooy, sx, sy, syaw0, syaw1, gx, gy, gyaw0, gyaw1)
                        scenarios_success += 1
                    end
                    scenarios_allnum += 1
                    println("!!!!Done!!!!")
                    println("success number: ",scenarios_success)
                    println("all number: ",scenarios_allnum)
                end
            end
        end
    end

    # lateral to parking place (20*20)
    for sy in 3.0:0.1:5.0  # [m]
        findmin = 0;
        for sx in -20.0:2:20.0 # [m]
            syaw0 = deg2rad(180.0)
            syaw1 = deg2rad(180.0)
            println("!!!!Start!!!!!")
            path = trailer_hybrid_a_star.calc_hybrid_astar_path(sx, sy, syaw0, syaw1, gx, gy, gyaw0, gyaw1, ox, oy,trailer_hybrid_a_star.XY_GRID_RESOLUTION, trailer_hybrid_a_star.YAW_GRID_RESOLUTION, h_dp, c, kdtree)
            if path != nothing
                plot_path(path, oox, ooy, sx, sy, syaw0, syaw1, gx, gy, gyaw0, gyaw1)
            else
                findmin = 1;
                break
            end
            println("!!!!Done!!!!")
        end
        if findmin == 0
            println("minimum distance to obstacles: ", sy - 3.0)
            break;
        end
    end

    # # longitudinal to parking place
    # # same direction to parking place (12*10)
    # for sy in 15.0:2:40.0  # [m]
    #     for sx in -20.0:2:0 # [m]
    #         syaw0 = deg2rad(90.0)
    #         syaw1 = deg2rad(90.0)
    #         println("!!!!Start!!!!!")
    #         path = trailer_hybrid_a_star.calc_hybrid_astar_path(sx, sy, syaw0, syaw1, gx, gy, gyaw0, gyaw1, ox, oy,trailer_ybrid_a_star.XY_GRID_RESOLUTION, trailer_hybrid_a_star.YAW_GRID_RESOLUTION, h_dp, c, kdtree)
    #         if path != nothing
    #             plot_path(path, oox, ooy, sx, sy, syaw0, syaw1, gx, gy, gyaw0, gyaw1)
    #             scenarios_success += 1
    #         end
    #         scenarios_allnum += 1
    #         println("!!!!Done!!!!")
    #         println("success number: ",scenarios_success)
    #         println("all number: ",scenarios_allnum)
    #     end
    # end

    # # longitudinal to parking place
    # # opposite direction to parking place (12*10)
    # for sy in 10.0:2:35.0  # [m]
    #     for sx in 0:2:20.0 # [m]
    #         syaw0 = deg2rad(90.0)
    #         syaw1 = deg2rad(90.0)
    #         println("!!!!Start!!!!!")
    #         path = trailer_hybrid_a_star.calc_hybrid_astar_path(sx, sy, syaw0, syaw1, gx, gy, gyaw0, gyaw1, ox, oy,trailer_hybrid_a_star.XY_GRID_RESOLUTION, trailer_hybrid_a_star.YAW_GRID_RESOLUTION, h_dp, c, kdtree)
    #         if path != nothing
    #             plot_path(path, oox, ooy, sx, sy, syaw0, syaw1, gx, gy, gyaw0, gyaw1)
    #             scenarios_success += 1
    #         end
    #         scenarios_allnum += 1
    #         println("!!!!Done!!!!")
    #         println("success number: ",scenarios_success)
    #         println("all number: ",scenarios_allnum)
    #     end
    # end
    
    println("success rate: ", scenarios_success/scenarios_allnum)
end


function plot_path(path, oox, ooy, sx, sy, syaw0, syaw1, gx, gy, gyaw0, gyaw1)
    cla()
    plot(oox, ooy, ".k")
    trailer_hybrid_a_star.trailerlib.plot_trailer(sx, sy, syaw0, syaw1, 0.0)
    trailer_hybrid_a_star.trailerlib.plot_trailer(gx, gy, gyaw0, gyaw1, 0.0)
    x = path.x
    y = path.y
    yaw = path.yaw
    yaw1 = path.yaw1
    direction = path.direction

    plot(x, y, "-r", label="Hybrid A* path")

    grid(true)
    axis("equal")
    name = string("movie/scenarios_result/", sx, "_", sy, "_", round(rad2deg(syaw0)), "_", round(rad2deg(syaw1)), ".png")
    savefig(name)
end


if length(PROGRAM_FILE)!=0 &&
    contains(@__FILE__, PROGRAM_FILE)
    main()
end

