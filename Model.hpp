//
//  Model.hpp
//  IA-GVRP
//
//  Created by Filip Cornell on 2017-06-11.
//  Copyright © 2017 Filip Cornell. All rights reserved.
//

#ifndef Model_hpp
#define Model_hpp

#include <stdio.h>
#include <queue>
#include <string>
#include <vector>
#include <ctime>
using namespace std;


/*
 Struct location is to store all the different locations possible. That is, the gas stations, the depot and the client. All locations in the problems have one of these.
 */
struct location { //I see all the locations as the same type.
    string name;
    char type;
    double latitude;
    double longitude;
    int index;
    
};

/*
 Struct stop contains a possible stop on the route. This is needed, as we can pass all the gasstations, and the depot, various times. However, in a correct solution there is only one stop at every client.
 */
struct stop {
    location loc;
    double fuelLeft;
    double distanceTravelled;
    double tau; //The time that has passed.
    vector<struct stop> clients;
    vector<struct stop> gasStations;
};

class Model {
private:
    
    
    
    
    /*
     
     PARAMETERS
     
     */
    
    int nrClientsVisited; //How many clients we have visited so far in the algorithm.
    double Q; //Fuel capacity
    double r; //Fuel rate consumption
    double TL; //TourLength
    double v; //Average velocity
    int m; //Number of vehicles
    int currentM; //current number of vehicles.
    double** distances; //Matrix with all the distances.
    double** t; //Traveltime
    vector<location> locations;
    /*
     
     VARIABLES
     
     */
    
    int** x; //Binary decision variable equal to 1 if a vehicle travels from vertex i to j and 0 otherwise
    double* y; //Fuel specifying the remaining tank fuel level upon arrival to vertex j. It is reset to Q at each refueling station vertex i and the depot. In the final implementation, this resulted not to be very necessary.
    double* tau; //Time variable specifying the time of arrival of a vehicle at vertex j, initialized to zero upon departure from the depot. In the final implementation, this resulted not to be very necessary.
    int nrLocations; //How many locations we have in total in the model.
    int nrGasStations; //How many gasstations we have in the problem
    int nrClients; //How many clients we have in the problem.
    double* p; //The service time/loading time we have at each location. I consider the depot not to have any service time, unless it works as a stop inbetween stations during a tour.
    
    
    
    /*
     
     FUNCTIONS
     
     */
    double degreesToRadian(double); //Turns degrees into radians to calculate distances.
    void calculateDistancesAndInitVar(); //This function calculates the distances and times between the different locations. It also initiates variables and such.
    void printStats(); //Function used for debugging. Prints all the information needed in different situations.
    long maxNrIterations; //THe maximum number of iterations, as instructed by the user.
    double goalFunction(); //Returns the distance that the problem has run so far.
    bool clientVisitedInTour(); //Makes sure we do not have any tours where we do not visit a client. If we do not have this, we will have tours in which we will might only will visit gasstations, and it will lead to solutions that lead to nothing and the program could possibly run forever, only adding tours with only gasstations.
    bool rest1(int); //restriction 1 in model
    bool rest4(struct stop, struct stop); //restriction 6 in model
    bool rest5(); //restriction 7 in model
    bool rest6(struct stop); //restriction 8 in model
    bool rest7(struct stop, struct stop); //restriction 9 in model
    bool rest8(struct stop); //restriction 10 in model
    bool rest9(struct stop, struct stop); //restriction 11 in model
    
    
    /*
     
     THINGS USED FOR THE SOLVING METHOD (FORWARD CHECKING + GRAPH BACKJUMPING)
     
     */
    
    /*
     
     Tools/Parameters
     
     */
    int nrSolutionsFound; //Counts how many solutions
    double bestSolutionFound; //Saves the value of the best solution found so far.
    bool* forwardCheckLocations; //IMPORTANT TOOL TO USE
    
    unsigned long nrIterations; //This nr keeps hold of the nr of iterations done so far. Should be long, or unsigned long.
    /*
     This struct is to maintain a solution and all the information needed to present.
     */
    struct solution {
        double goalFunctionAnswer;
        unsigned int nrIterations;
        vector<struct stop> stops; //The route of the solution.
    };
    
   
    struct solution bestSolution; //The best Solution!
    
    vector<struct location> clients; //vector that contains all the clients in the problem.
    vector<struct location> gasStations; //vector containing all the gasstations in the problem.
    
    
    clock_t startTime;
    
    /*
     
     FOR SORTING THE VARIABLES
     
     
     */
    
    
    /*
     
     FUNCTIONS
     
     */
    
    vector<struct stop> forwardChecking(struct stop, bool); //This function creates the domain for the next iteration.
    bool moveOn(struct stop, struct stop); //Checks a number of restrictions to make sure we can move on to the next iteration.
    bool solutionFound(); //Checks whether we have found a solution.
    void newIteration(struct stop); //Performs a new iteration, where the struct stop is the new stop in the iteration.
    void undoIteration(vector<struct stop>); //Used to undo the previous iteration in a secure way, to make sure we we can move on without a problem to the next iteration. 
    bool sortVariables(); //Sorts the variables in an order to find the solution the quickest way possible.
    vector<struct stop> road; //The current the trucks have run so far in the algorithm. Simply the current path.
    
    string fileName;
    
    
public:
    void deleteAllVariables(); //Deletes all the variables to avoid memory leaks.
    Model(queue<string>,string); //Create the model and initiates variables for implementing the algorithm.
    void solve(); //starts solving the model to find a solution.
    
    void setMaxNrIter(long);
    
    
};



#endif /* Model_hpp */
