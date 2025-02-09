#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if (!is)
        return std::nullopt;

    auto size = is.tellg();
    std::vector<std::byte> contents(size);

    is.seekg(0);
    is.read((char *)contents.data(), size);

    if (contents.empty())
        return std::nullopt;
    return std::move(contents);
}

bool checkRange(int user_input_x, int user_input_y)
{
    // check here if the input is in the range [0, 100]
    if (user_input_x >= 0 && user_input_x <= 100 && user_input_y >= 0 && user_input_y <= 100)
        return true;
    return false;
}

int getInput()
{
    // this function asks the user to get the input and shows some output in return
    int input;
    std::cin >> input;
    return input;
}

int main(int argc, const char **argv)
{
    std::string osm_data_file = "";
    if (argc > 1)
    {
        for (int i = 1; i < argc; ++i)
            if (std::string_view{argv[i]} == "-f" && ++i < argc)
                osm_data_file = argv[i];
    }
    else
    {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }

    std::vector<std::byte> osm_data;

    if (osm_data.empty() && !osm_data_file.empty())
    {
        std::cout << "Reading OpenStreetMap data from the following file: " << osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if (!data)
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }

    float start_x = -1, start_y = -1, end_x = -1, end_y = -1;
    while (!checkRange(start_x, start_y))
    {
        std::cout << "Please Input starting point x, y in range [0,100]"
                  << "\n";
        start_x = getInput();
        start_y = getInput();
    }
    while (!checkRange(end_x, end_y))
    {
        std::cout << "Please Input destination x, y in range [0,100]"
                  << "\n";
        end_x = getInput();
        end_y = getInput();
    }

    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface &surface)
                                 { surface.dimensions(surface.display_dimensions()); });
    display.draw_callback([&](io2d::output_surface &surface)
                          { render.Display(surface); });
    display.begin_show();
}
