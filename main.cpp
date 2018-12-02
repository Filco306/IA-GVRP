//
//  main.cpp
//  IA-GVRP
//  Best version so far, that solves the problem very slowly.
// The other version was faster due to not creating the domain in two levels.
// To change to the other, more rapid sorting, follow the instrucions in the README.
//  Created by Filip Cornell on 2017-06-10.
//  Copyright © 2017 Filip Cornell. All rights reserved.
//

#include <iostream>
#include <string>
#include <fstream>
#include <queue>
#include <string>
#include "Model.hpp"
using namespace std;
int main(int argc, const char * argv[]) {
    
    cout << "Enter filename: ";
    string fileName;
    cin >> fileName;
    
    
    

    ifstream infile(fileName);
    while (true) {
        if (infile) {
            cout << "File found. " << endl;
            break;
        } else {
            cout << "ERROR 404: File not Found." << endl;
        }
        cout << "Enter filename: " << endl;
        cin >> fileName;
        if (fileName == "0")
            return false;
        
        infile.open(fileName);
    }
    
    string line;
    
    queue<string> lines = *new queue<string>;
    while (getline(infile,line)) {
        lines.push(line);
    }
    
    Model m = *new Model(lines,fileName);
    long maxNrIterations;
    cout << "Enter the maximum number of iterations: ";
    cin >> maxNrIterations;
    m.setMaxNrIter(maxNrIterations);
    cout << "Solving problem..." << endl;
    
    m.solve();
    m.deleteAllVariables();
    return 0;
}
