#include <fstream>
#include <vector>
#include <cstdlib>
#include <iostream>

int main() {
    std::ifstream ifile("input.txt", std::ios::in);
    std::vector<double> scores;

    //check to see that the file was opened correctly:
    if (!ifile.is_open()) {
        std::cerr << "There was a problem opening the input file!\n";
        exit(1);//exit or do additional error checking
    }

    double num = 0.0;
    //keep storing values from the text file so long as data exists:
    while (ifile >> num) {
        scores.push_back(num);
    }

    //verify that the scores were stored correctly:
    for (int i = 0; i < scores.size(); ++i) {
        std::cout << scores[i] << std::endl;
    }

    return 0;
}
