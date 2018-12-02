//
//  Model.cpp
//  IA-GVRP
//
//  Created by Filip Cornell on 2017-06-11.
//  Copyright © 2017 Filip Cornell. All rights reserved.
//

/*
 The implementation of the model. In the end, I decided to implement the algorithm here aswell. Therefore, I have both the model and the
 
 
 
 For the explanations of the purposes of each function, look in Model.hpp.
 */



#include "Model.hpp"
#include <string>
#include <cmath>
#include <iostream>
#include <algorithm>

using namespace std;
void Model::setMaxNrIter(long n) {
    maxNrIterations = n;
}


void Model::printStats () {
    cout << "ITERATION " << nrIterations << endl << endl;
    int cnt = 0;
    for (int i = 0; i < nrLocations; i++) {
        for (int j = 0; j < nrLocations; j++) {
            if (x[i][j] == 1) {
                cnt++;
            }
        }
    }
    cout << "Amount of variables set: " << cnt << endl;
    cout << "Amount of variables set, including dummy vertices: " << road.size() << endl;
    cout << "Goalfunction: " << goalFunction() << endl;
    cout << "PRINTING ALL X VARIABLES." << endl << endl;
    cout << "   ";
    
    for (int i = 0; i < nrLocations; i++) {
        cout << i << " ";
    }
    cout << endl << "   ";
    /*for (int i = 0; i < nrLocations; i++) {
     cout << columnjCovered[i] << " ";
     }*/
    cout << endl;
    cout << "   ";
    for (int i = 0; i < nrLocations; i++)
        cout << "__";
    cout << endl;
    for (int i = 0; i < nrLocations; i++) {
        cout << i << " ";
        if (i < 10) {
            cout << " ";
        }
        
        for (int j = 0; j < nrLocations; j++) {
            if (x[i][j] != -1)
                cout << x[i][j] << " ";
            else
                cout << "X ";
        }
        cout << endl;
    }
    cout << "Printing distances parameters" << endl << endl;
    for (int i = 0; i < nrLocations; i++) {
        for (int j = 0; j < nrLocations; j++) {
            cout << distances[i][j] << " ";
        }
        cout << endl;
    }
    
    cout << "Printing time parameters" << endl << endl;
    for (int i = 0; i < nrLocations; i++) {
        for (int j = 0; j < nrLocations; j++) {
            cout << t[i][j] << " ";
        }
        cout << endl;
    }
    cout << endl << endl;
}


double Model::degreesToRadian(double deg) {
    return deg * M_PI / 180.0;
}



void Model::calculateDistancesAndInitVar() {
    distances = new double*[nrLocations];
    
    t = new double*[nrLocations];
    x = new int*[nrLocations];
    tau = new double[nrLocations];
    y = new double[nrLocations];
    p = new double[nrLocations];
    y[0] = Q;
    forwardCheckLocations = new bool[nrLocations];
    nrSolutionsFound = 0;
    
    
    for (int i = 0; i < nrLocations;i++) {
        if (i < nrGasStations) {
            p[i] = 0.25;
        } else {
            p[i] = 0.5;
        }
        
        forwardCheckLocations[i] = false;
        
        
        distances[i] = new double[nrLocations];
        t[i] = new double[nrLocations];
        x[i] = new int[nrLocations];
        
        if (i >= nrGasStations) { //Here, we fill the two vectors up that we will use to
            clients.push_back(locations[i]);
        } else if (i < nrGasStations) { // We shall not have any other gas stations in the
            gasStations.push_back(locations[i]);
            
        }
        
        for (int j = 0; j < locations.size(); j++) {
            
            
            
            
            if (i == j) { //Which one is columns, which one is rows? i has to be rows, j columns
                distances[i][j] = -1.0;
                t[i][j] = -1.0;
                x[i][j] = -1;
            } else if ((i < nrGasStations and 1 <= i) and (j < nrGasStations and 1 <= j)) { //We are not interested in distances between different tank stations.
                distances[i][j] = -1.0;
                t[i][j] = -1.0;
                x[i][j] = -1;
            } else { //calculate distance
                double lat1 = locations[i].latitude;
                double lat2 = locations[j].latitude;
                double lon1 = locations[i].longitude;
                double lon2 = locations[j].longitude;
                double radiusOfEarth = 4182.44949;
                double dLat = degreesToRadian(lat2-lat1);
                double dLon = degreesToRadian(lon2-lon1);
                
                double a = sin(dLat/2)*sin(dLat/2) + cos(degreesToRadian(lat1))*cos(degreesToRadian(lat2))*sin(dLon/2)*sin(dLon/2);
                double c = 2 * atan2(sqrt(a), sqrt(1-a));
                distances[i][j] = radiusOfEarth * c;
                
                t[i][j] = distances[i][j]/v;
                x[i][j] = 0;
            }
        }
        
    }
    
    
    
    
    x[0][1] = -1;
    x[1][0] = -1;
    
    printStats();
}

double Model::goalFunction() { //This one is wrong.
    
    if (road.size() > 0) {
        return road.back().distanceTravelled;
    } else {
        return 0;
    }
    
}
Model::Model(queue<string> q, string fileName1) {
    
    string fstLine = q.front();
    string type1 = "Type";
    
    fileName = fileName1;
    if (fstLine.find(type1) != string::npos) {
        q.pop(); //Popping first line of data that only species what is in the columns.
    }
    nrLocations = 0;
    locations = *new vector<location>;
    nrGasStations = 0; //Reset nrGasStations
    nrClients = 0;
    nrClientsVisited = 0;
    while (q.front().size() > 1) {
        
        cout << q.front() << endl;
        nrLocations++;
        string nName;
        char nType;
        double latitude;
        double longitude;
        
        string currentLine = q.front();
        q.pop(); //Popping the depot!
        
        
        
        int count = 1;
        
        while (currentLine[count] != '\t' and currentLine[count] != ' ') {//Heading for spaces and tabs
            count++;
        }
        
        nName = currentLine.substr(0,count);
        
        while (currentLine[count] == '\t' or currentLine[count] == ' ') { //Heading FROM spaces and tabs
            count++;
        }
        
        nType = currentLine[count];
        switch (nType) {
            case 'd':
            case 'f':
                nrGasStations++;
                break;
            case 'c':
                nrClients++;
                break;
            default:
                break;
        }
        count++;
        while (currentLine[count] == '\t' or currentLine[count] == ' ') { //Heading FROM spaces and tabs
            count++;
        }
        int indexFirst = count;
        while (currentLine[count] != '\t' and currentLine[count] != ' ') { //Heading for spaces and tabs
            count++;
        }
        int indexSecond = count;
        
        longitude = stod(currentLine.substr(indexFirst,indexSecond-indexFirst));
        while (currentLine[count] == '\t' or currentLine[count] == ' ') { //Heading FROM spaces and tabs
            count++;
        }
        
        indexFirst = count;
        
        latitude = stod(currentLine.substr(indexFirst,currentLine.size()-indexFirst));
        
        location newLoc;
        newLoc.latitude = latitude;
        newLoc.longitude = longitude;
        newLoc.name = nName;
        newLoc.type = nType;
        newLoc.index = nrLocations-1;
        locations.push_back(newLoc);
        
    }
    
    
    
    nrLocations = (int)locations.size();
    
   
    
    
    
    q.pop();
    
    //Parameters
    string a = q.front();
    int index;
    if (a[a.size()-1] == '/') {
        index = (int)a.size()-2;
    } else {
        index = (int)a.size()-3;
    }
    
    
    cout << a << endl;
    while (a[index] != '/') {
        index--;
    }
    index++;
    if (a[a.size()-1] == '/') {
        Q = stod(a.substr(index,a.size()-1-index));
    } else {
        Q = stod(a.substr(index,a.size()-2-index));
    }
    cout << q.front() << endl;
    q.pop();
    
    a = q.front();
    
    if (a[a.size()-1] == '/') {
        index = (int)a.size()-2;
    } else {
        index = (int)a.size()-3;
    }
    
    while (a[index] != '/') {
        index--;
    }
    index++;
    if (a[a.size()-1] == '/') {
        r = stod(a.substr(index,a.size()-1-index));
    } else {
        r = stod(a.substr(index,a.size()-2-index));
    }
    
    q.pop();
    
    a = q.front();
    if (a[a.size()-1] == '/') {
        index = (int)a.size()-2;
    } else {
        index = (int)a.size()-3;
    }
    
    
    while (a[index] != '/') {
        index--;
    }
    index++;
    if (a[a.size()-1] == '/') {
        TL = stod(a.substr(index,a.size()-1-index));
    } else {
        TL = stod(a.substr(index,a.size()-2-index));
    }
    q.pop();
    
    a = q.front();
    if (a[a.size()-1] == '/') {
        index = (int)a.size()-2;
    } else {
        index = (int)a.size()-3;
    }
    
    
    while (a[index] != '/') {
        index--;
    }
    index++;
    if (a[a.size()-1] == '/') {
        v = stod(a.substr(index,a.size()-1-index));
    } else {
        v = stod(a.substr(index,a.size()-2-index));
    }
    q.pop();
    
    a = q.front();
    if (a[a.size()-1] == '/') {
        index = (int)a.size()-2;
    } else {
        index = (int)a.size()-3;
    }
    
    
    while (a[index] != '/') {
        index--;
    }
    index++;
    if (a[a.size()-1] == '/') {
        m = stod(a.substr(index,a.size()-1-index));
    } else {
        m = stod(a.substr(index,a.size()-2-index));
    }
    
    q.pop();
    
    
    
    
    calculateDistancesAndInitVar();
    
    
    
    m = 10;
}




bool Model::rest1(int i) {
    int total = 0;
    if (i >= nrGasStations) {
        
        for (int j = 0; j < nrLocations; j++) {
            if (x[i][j] >= 1) {
                total++;
                if (total > 1) {
                    return false;
                }
            }
        }
        
        return (total == 1);
    } else {
        return true;
    }
}

bool Model::rest4(struct stop prev, struct stop curr) {
    
    int i = prev.loc.index;
    int j = curr.loc.index;
    double tauj = curr.tau;
    double taui = prev.tau;
    if (i != 0 and j != 0 and i != j) {
        return (tauj >= taui + (t[i][j] - p[j])*x[i][j] - TL*(1-x[i][j]));
    } else {
        
        return true;
    }
    
    
    return true;
}
bool Model::rest5() {
    return (tau[0] >= 0.0 and tau[0] <= TL);
}
bool Model::rest6(struct stop curr) {
    
    int j = curr.loc.index;
    double tauj = curr.tau;
    if (j != 0) {
        double t0j = t[0][j];
        
        bool first = (t0j <= tauj);
        bool snd = (tauj <= TL - (t[j][0] + p[j]));
        return (first and snd);
    } else {
        
        return true;
    }
    
    
}

bool Model::rest7(struct stop prev, struct stop curr) {
    
    int i = prev.loc.index;
    int j = prev.loc.index;
    
    double yi = prev.fuelLeft;
    double yj = curr.fuelLeft;
    if (j > nrGasStations) {
        return (yj <= yi - r*distances[i][j]*x[i][j] + Q*(1-x[i][j]));
    } else {
        
        return true;
    }
    
    
    return true;
}


bool Model::rest8(struct stop curr) {
    
    int j = curr.loc.index;
    if (j >= nrGasStations or curr.fuelLeft == Q) {
        return true;
    } else
        return false;
    
    return true;
}
bool Model::rest9(struct stop prev, struct stop curr) {
    
    int j = prev.loc.index;
    int l = curr.loc.index;
    
    double yj = curr.fuelLeft;
    
    if (j > nrGasStations and l <= nrGasStations) {
        
        if (r*distances[j][0] < r*(distances[j][l] + distances[l][0])) {
            return yj >= r*distances[j][0];
        } else {
            return yj >= r*(distances[j][l] + distances[l][0]);
        }
        
        
    } else {
        return true;
    }
    
    
    
}

bool Model::clientVisitedInTour () {
    //Check for unnecessary tours
    
    bool clientVisitedInTour = false;
    
    for (int i = 1; i < road.size(); i++) {
        
        if (road[i].loc.index >= nrGasStations) { //If we have a client, the tour has a client
            clientVisitedInTour = true;
        }
        
        if (road[i].loc.index == 0) {
            if (!clientVisitedInTour) {
                return false;
            } else {
                clientVisitedInTour = false;
            }
        }
    }
    return true;
}


bool Model::solutionFound() {
    /*
     
     Check all restrictions
     
     */
    
    if (nrClientsVisited < nrClients) {
        return false;
    }
    if (road[road.size()-1].loc.index != 0) {
        return false;
    }
    
    if (!rest5()) {
        return false;
    }
    
    for (int i = 0; i < nrLocations; i++) {
        
        if (!rest1(i)) {
            return false;
        }
        
        
        
        
        
    }
    
    
    
    if (!rest6(road[0]) or !rest8(road[0])) {
        return false;
    }
    
    for (int i = road.size()-1; i >= 1; i--) {
        if (!rest6(road[i]) or !rest8(road[i])) {
            return false;
        }
        
        if (!rest4(road[i-1], road[i])){
            return false;
        }
        if( !rest7(road[i-1], road[i])) {
            return false;
        }
        if(!rest9(road[i-1], road[i])) {
            return false;
        }
        
    }
    
    
    return true;
}

//Checks if we can moveOn by changing all the variables, by after that
bool Model::moveOn(struct stop currentStop, struct stop nextStop) { //Something wrong in MoveOn
    
    if (currentM >= m) {
        return false;
    }
    
    //Set variables first
    int i = currentStop.loc.index;
    int j = nextStop.loc.index;
    
    x[i][j] = 1;
    double yj = y[j];
    double tauj = tau[j];
    
    if (j < nrGasStations) {
        y[j] = Q;
    } else {
        y[j] = nextStop.fuelLeft;
    }
    
    tau[j] = nextStop.tau;
    
    //We test the following restrictions: 1,4,6,7,8,9. However, I must change how I control
    
    
    if (!rest1(i)) {
        x[i][j] = 0;
        y[j] = yj;
        tau[j] = tauj;
        return false;
    } else if (!rest4(currentStop, nextStop)) {
        x[i][j] = 0;
        y[j] = yj;
        tau[j] = tauj;
        return false;
    } else if (!rest6(nextStop)) {
        x[i][j] = 0;
        y[j] = yj;
        tau[j] = tauj;
        return false;
    } else if (!rest7(currentStop, nextStop)) {
        x[i][j] = 0;
        y[j] = yj;
        tau[j] = tauj;
        return false;
    } else if (!rest8(nextStop)) {
        x[i][j] = 0;
        y[j] = yj;
        tau[j] = tauj;
        return false;
    } else if (!rest9(currentStop, nextStop)){
        x[i][j] = 0;
        y[j] = yj;
        tau[j] = tauj;
        return false;
    }
    
    
    
    
    
    //Then test it against restrictions. If not valid -> return false and reset all the variables.
    
    
    return true;
}

bool cmp (struct stop i, struct stop j) {
    //First, compare how many the one can visit. This should still be quite a lot and they should all be the same practically.
    //Secondly, compare the amount of time left.
    // Thridly, we sort by how much gas is left in tank (not useful for sorting the gas stations)
    
    if (i.clients.size() + i.gasStations.size() != j.clients.size() + j.gasStations.size()) { //Menor a mayor dominio.
        return (i.clients.size() + i.gasStations.size() < j.clients.size() + j.gasStations.size());
    }
    
    if (i.tau != j.tau) {
        return (i.tau < j.tau);
    }
    
    
    return (i.fuelLeft > j.fuelLeft);
    
}


/*
 
 The function that recursively calls itself to perform a new iteration. The baseline of the algorithm.
 
 */
void Model::newIteration(struct stop currentStop) {
    if (nrIterations < maxNrIterations) {
        
        
        if (nrIterations % 100000 == 0) {
            cout <<"Test "<< fileName<<" at iteration "<< nrIterations << ": ";
            for (int i = 0; i < road.size(); i++) {
                if (i != road.size() - 1) {
                    if (road[i].loc.name[0] == ' ') {
                        string print = road[i].loc.name.substr(1);
                        cout << print << "-";
                    } else
                        cout << road[i].loc.name << "-";
                } else {
                    if (road[i].loc.name[0] == ' ') {
                        string print = road[i].loc.name.substr(1);
                        cout << print;
                    } else
                        cout << road[i].loc.name;
                }
            }
            cout << endl;
            cout << "Goalfunction: " << goalFunction()<< endl;
            cout << "This took " << double(clock() - startTime)/CLOCKS_PER_SEC << " seconds. ";
        }
        
        nrIterations++;
        
        
        
        
        currentStop.clients = forwardChecking(currentStop, true);
        
        for (int i = 0; i < currentStop.clients.size(); i++) {
            currentStop.clients[i].clients = forwardChecking(currentStop.clients[i], true);
            currentStop.clients[i].gasStations = forwardChecking(currentStop.clients[i], false);
        }
        
        sort(currentStop.clients.begin(),currentStop.clients.end(),cmp);
        
        
        
        currentStop.gasStations = forwardChecking(currentStop, false);
        
        
        for (int i = 0; i < currentStop.gasStations.size(); i++) {
            currentStop.gasStations[i].clients = forwardChecking(currentStop.gasStations[i], true);
            currentStop.gasStations[i].clients = forwardChecking(currentStop.gasStations[i], false);
        }
        
        
        if (currentStop.gasStations.size() > 1 and currentStop.gasStations[0].loc.index == 0){
            sort(currentStop.gasStations.begin()+1,currentStop.gasStations.end(), cmp);
        } else {
            sort(currentStop.gasStations.begin(),currentStop.gasStations.end(), cmp);
        }
        road.push_back(currentStop);
        
        forwardCheckLocations[currentStop.loc.index] = true;
        if (currentStop.loc.index == 0 and solutionFound()) {
            nrSolutionsFound++;
            
            
            
            
            
            struct solution newSol;
            
            
            
            
            
            newSol.stops = *new vector<struct stop>;
            newSol.goalFunctionAnswer = goalFunction();
            
            
            
            
            for (int i = 0; i < road.size(); i++) {
                struct stop cStop = road[i];
                newSol.stops.push_back(cStop);
            }
            
            if (newSol.goalFunctionAnswer < bestSolution.goalFunctionAnswer or bestSolution.goalFunctionAnswer == -1.0) {
                
                bestSolution.stops.clear();
                bestSolution = newSol;
                cout << "Test " << fileName << ": SOLUTION FOUND. SOLUTION NR" << nrSolutionsFound << ". " << "at iteration " << nrIterations << " with min z = " << bestSolution.goalFunctionAnswer << ": ";
                for (int i = 0; i < road.size(); i++) {
                    if (i != road.size() - 1) {
                        if (road[i].loc.name[0] == ' ') {
                            string print = road[i].loc.name.substr(1);
                            cout << print << "-";
                        } else
                            cout << road[i].loc.name << "-";
                    } else {
                        if (road[i].loc.name[0] == ' ') {
                            string print = road[i].loc.name.substr(1);
                            cout << print;
                        } else
                            cout << road[i].loc.name;
                    }
                }
                cout << endl;
                cout << "This took " << double(clock() - startTime)/CLOCKS_PER_SEC << " seconds. ";
            }
            
            
            
            
        } else {
            if (currentM < m) { //This is not bad probably.
                for (int j = 0; j < currentStop.clients.size(); j++) {
                    if (moveOn(currentStop, currentStop.clients[j]) and nrIterations < maxNrIterations) {
                        //road.push_back(currentStop.clients[j]);
                        nrClientsVisited++;
                        newIteration(currentStop.clients[j]);
                        nrClientsVisited--;
                        undoIteration(road);
                    }
                }
                if (currentStop.gasStations.size() > 1) {
                    for (int j = 1; j < currentStop.gasStations.size(); j++) {
                        if (moveOn(currentStop, currentStop.gasStations[j]) and nrIterations < maxNrIterations) {
                            //road.push_back(currentStop.gasStations[j]);
                            bool addedM = false;
                            if (currentStop.gasStations[j].loc.index == 0) {
                                currentM++;
                                addedM = true;
                            }
                            newIteration(currentStop.gasStations[j]);
                            if (addedM) {
                                currentM--;
                            }
                            undoIteration(road);
                        }
                    }
                    
                    
                    
                    if (moveOn(currentStop, currentStop.gasStations[0]) and nrIterations < maxNrIterations) {
                        //road.push_back(currentStop.gasStations[j]);
                        bool addedM = false;
                        if (currentStop.gasStations[0].loc.index == 0) {
                            currentM++;
                            addedM = true;
                        }
                        newIteration(currentStop.gasStations[0]);
                        if (addedM) {
                            currentM--;
                        }
                        undoIteration(road);
                    }
                } else {
                    for (int j = 0; j < currentStop.gasStations.size(); j++) {
                        if (moveOn(currentStop, currentStop.gasStations[j]) and nrIterations < maxNrIterations) {
                            
                            bool addedM = false;
                            if (currentStop.gasStations[j].loc.index == 0) {
                                currentM++;
                                addedM = true;
                            }
                            newIteration(currentStop.gasStations[j]);
                            if (addedM) {
                                currentM--;
                            }
                            undoIteration(road);
                        }
                    }
                }
            }
        }
    }
}



vector<struct stop> Model::forwardChecking(struct stop currentStop,bool clients1) {
    vector<struct stop> stops = *new vector<struct stop>;
    int i = currentStop.loc.index;
    if (clientVisitedInTour()) {
        if (clients1) {
            
            for (int j = 0; j < clients.size(); j++) { //Checking the domain, eliminating those not possible to travel to.
                if (currentStop.loc.index != clients[j].index and x[i][clients[j].index] != -1) {
                    
                    struct stop newStop;
                    newStop.loc = clients[j];
                    if (currentStop.loc.index != 0) {
                        newStop.tau = currentStop.tau + t[i][clients[j].index] + p[i];
                        newStop.fuelLeft = currentStop.fuelLeft - distances[i][clients[j].index]*r;
                        newStop.distanceTravelled = currentStop.distanceTravelled + distances[i][clients[j].index]; //Not sure if necessary
                    } else {
                        newStop.tau = 0.0 + t[i][clients[j].index] + p[i];
                        if (road.size() == 1) { //If we ONLY have the beginning; that is, if we start from the start, we have to remove the p[i]
                            newStop.tau -= p[0];
                        }
                        newStop.fuelLeft = Q - distances[i][clients[j].index]*r;
                        newStop.distanceTravelled = currentStop.distanceTravelled + distances[i][clients[j].index];
                    }
                    if (newStop.fuelLeft >= 0.0 and newStop.tau < (double)TL and !forwardCheckLocations[clients[j].index]) { //forwardCheckLocations has to be altered
                        if (newStop.loc.index == 0) { //IF this is the home depot
                            newStop.tau = 0.0; //Then it would be starting over.
                        }
                        stops.push_back(newStop);
                    }
                }
            }
        } else {
            for (int j = 0; j < gasStations.size(); j++) { //Checking the domain, eliminating those not possible to travel to.
                if (i != gasStations[j].index and x[i][gasStations[j].index] != -1) {
                    
                    
                    struct stop newStop;
                    newStop.loc = gasStations[j];
                    
                    
                    if (currentStop.loc.index != 0) { //Shouldnt it be newStop here?
                        newStop.tau = currentStop.tau + t[i][gasStations[j].index] + p[i];
                        newStop.fuelLeft = Q; //If it is a gas station, we will fill it up completely again.
                        newStop.distanceTravelled = currentStop.distanceTravelled + distances[i][gasStations[j].index]; //Not sure if necessary
                    } else {
                        newStop.tau = 0.0 + t[i][gasStations[j].index] + p[i];
                        if (road.size() == 1) { //If we ONLY have the beginning; that is, if we start from the start, we have to remove the p[i]
                            newStop.tau -= p[0];
                        }
                        newStop.fuelLeft = Q;
                        newStop.distanceTravelled = currentStop.distanceTravelled + distances[i][gasStations[j].index];
                    }
                    
                    
                    
                    
                    if (newStop.fuelLeft >= 0.0 and newStop.tau < (double)TL) { //forwardCheckLocations is not checked here, as these stations can be visited various times
                        stops.push_back(newStop);
                    }
                }
            }
        }
    }
    return stops;
}

void Model::solve () {
    
    
    
    /*
     
     First, we do the forward checking. We check all the variables, and we sort them
     
     */
    road = *new vector<struct stop>;
    struct stop currentStop;
    currentStop.loc = locations[0];
    currentStop.distanceTravelled = 0.0;
    currentStop.fuelLeft = Q;
    currentStop.tau = 0.0 - p[0]; //We subtract with p[0] to compensate for the fault it amke in the function forwardChecking
    
    
    bestSolution.goalFunctionAnswer = -1;
    
    
    currentStop.clients = forwardChecking(currentStop, true);
    
    for (int i = 0; i < currentStop.clients.size(); i++) {
        currentStop.clients[i].clients = forwardChecking(currentStop.clients[i], true);
        currentStop.clients[i].gasStations = forwardChecking(currentStop.clients[i], false);
    }
    
    
    sort(currentStop.clients.begin(),currentStop.clients.end(),cmp); 
    
    startTime = clock();
    currentStop.gasStations = forwardChecking(currentStop, false);
    
    for (int i = 0; i < currentStop.gasStations.size(); i++) {
        currentStop.gasStations[i].clients = forwardChecking(currentStop.gasStations[i], true);
        currentStop.gasStations[i].clients = forwardChecking(currentStop.gasStations[i], false);
    }
    
    sort(currentStop.gasStations.begin()+1,currentStop.gasStations.end(), cmp);
    
    currentStop.tau = 0.0; //Setting it to a correct value
    road.push_back(currentStop);
    forwardCheckLocations[currentStop.loc.index] = true;
    
    
    for (int j = 0; j < currentStop.clients.size(); j++) {
        if (moveOn(currentStop, currentStop.clients[j]) and nrIterations < maxNrIterations) {
            
            nrIterations++;
            nrClientsVisited++;
            
            newIteration(currentStop.clients[j]);
            
            nrClientsVisited--;
            undoIteration(road);
        }
    }
    for (int j = 0; j < currentStop.gasStations.size(); j++) {
        if (moveOn(currentStop, currentStop.gasStations[j]) and nrIterations < maxNrIterations) {
            
            nrIterations++;
            newIteration(currentStop.gasStations[j]);
            undoIteration(road);
        }
    }
    
    
    cout << "Best solution found: " << bestSolution.goalFunctionAnswer <<endl;
    for (int i = 0; i < bestSolution.stops.size(); i++) {
        if (road[i].loc.name[0] == ' ') {
            string print = bestSolution.stops[i].loc.name.substr(1);
            cout << print;
        } else
            cout << bestSolution.stops[i].loc.name;
        
        if (i != bestSolution.stops.size()-1) {
            cout << "-";
        }
    }
    cout << endl;
}

void Model::undoIteration(vector<struct stop> vect) {
    
    struct stop prevStop = road.at(vect.size()-2);
    struct stop removeStop = road.back();
    
    
    forwardCheckLocations[removeStop.loc.index] = false;
    int i = prevStop.loc.index;
    int j = removeStop.loc.index;
    
    x[i][j] = 0;
    road.pop_back();
    
    
}

void Model::deleteAllVariables() {
    delete [] tau;
    delete [] y;
    delete [] p;
    for (int i = 0; i < nrLocations; i++) {
        delete [] distances[i];
        delete [] x[i];
        delete [] t[i];
    }
}
