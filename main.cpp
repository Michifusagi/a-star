#include <iostream>
#include "source/AStar.hpp"

int main()
{
    AStar::Generator generator;
    generator.setWorldSize({25, 25});
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(true);
    generator.addCollision({10, 10});
    generator.addCollision({11, 10});
    generator.addCollision({9, 10});

    std::cout << "Generate path ... \n";
    auto path = generator.findPath({0, 0}, {20, 20});

    for(auto& coordinate : path) {
        std::cout << coordinate.x << " " << coordinate.y << "\n";
    }
}