#include "solver.h"
#include "instance.h"
#include <tuple>
ILOSTLBEGIN
#define pos(u,v) min(u,v)][max(u,v)

Solver::Solver(Instance* instance, string input, string output, double time_limit, double mem_limit)
	:instance(instance), inputFile(input), outputFile(output), time_limit(time_limit), mem_limit(mem_limit) {
	cerr << "-- Solving \n";
	startTime = chrono::high_resolution_clock::now();
	outputFile = instance->instanceName;
	gap = 1e5;
	status = "-";

	/*  SET -------------------------------- */
	for (int i = 0; i <= instance->num_nodes; ++i)
		N.push_back(i);
	for (int i = 1; i <= instance->num_nodes; ++i)
		C.push_back(i);
	arr.resize(instance->num_nodes+1, -1);
	arr[0] = 0;
	int tg1;
	for (int i = 1; i <= instance->num_nodes; i++)
	{
		arr[i] = i;
	}
	for (int i = 1; i <= instance->num_nodes; i++)
	{
		if (instance->time_drone[i] < 0) continue;
		for (int j = i + 1; j <= instance->num_nodes; j++) {
			if (instance->time_drone[j] < 0) continue;
			if (instance->time_drone[arr[i]] < instance->time_drone[arr[j]]) {
				tg1 = arr[i];
				arr[i] = arr[j];
				arr[j] = tg1;
			}
		}
	}



	for (int i = 1; i <= instance->num_nodes; i++)
	{
		cout << arr[i] << endl;
	}

	// --
	numNode = instance->num_nodes;
	numUAV = instance->num_drones;
	UB = 1e5;
	UB_tsp = 1e5;

	truckOnly = instance->truckonlyCustomer;
	freeCustomers = instance->freeCustomer;
	freeCustomers0 = instance->freeCustomer;
	freeCustomers0.insert(0);
}

Solver::~Solver() {
	//    cerr << "Runtime = " << (double)(clock() - startTime) / CLOCKS_PER_SEC << "s\n";
}

void Solver::Solve() {
	try {
		createModel();

		masterCplex.exportModel("lpex.lp");
		masterCplex.setParam(IloCplex::Param::Parallel, 1);
		masterCplex.setParam(IloCplex::Param::Threads, 16);
		masterCplex.setParam(IloCplex::TiLim, time_limit);
		masterCplex.setParam(IloCplex::TreLim, mem_limit);
		masterCplex.setParam(IloCplex::Param::MIP::Strategy::RINSHeur, 10);

		masterCplex.solve();
		if (masterCplex.getStatus() == IloAlgorithm::Infeasible) {
			cout << UB << endl;
			cout << "Infeasible" << endl;
		}
		else if (masterCplex.getStatus() == IloAlgorithm::Unbounded) {
			cout << "Unbounded" << endl;
		}
		else if (masterCplex.getStatus() == IloAlgorithm::Unknown) {
			cout << "Unknown" << endl;
		}
		else {
			cout << "DONE ..." << endl;
			cout << masterCplex.getObjValue() << endl;
			dispay_solution();
		}


	}
	catch (IloException& e) {
		cerr << "Conver exception caught: " << e << endl;
	}
	catch (...) {
		cerr << "Unknown exception caught" << endl;
	}


	auto endTime = chrono::high_resolution_clock::now();
	runtime = chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count();
	runtime = runtime / 1000;


	write_output();
	cout << "Environment cleared \n";
	//        workerCplex.end();
	masterCplex.end();

	//        workerEnv.end();
	masterEnv.end();
}

void Solver::createModel() {

	

	masterModel = IloModel(masterEnv);
	masterCplex = IloCplex(masterEnv);

	x = NumVar3D(masterEnv, numNode+1); // x_ijk
	h = NumVar3D(masterEnv, numNode+1); // h_ijk
	a = IloNumVarArray(masterEnv, numNode+1);
	z = IloNumVarArray(masterEnv, numNode+1);

	stringstream name;
	// x_ijk
	for (int i : N)
	{
		x[i] = NumVar2D(masterEnv, numNode+1);
		for (int j : N) {
			if (j == i) continue;
			x[i][j] = IloNumVarArray(masterEnv, numNode+1);
			for (int k : N) {
				name << "x." << i << "." << j << "." << k;
				x[i][j][k] = IloNumVar(masterEnv, 0, 1, ILOINT, name.str().c_str());
				name.str("");
			}
		}
	}


	// h_ijk
	for (int i : N)
	{
		h[i] = NumVar2D(masterEnv, numNode+1);
		for (int j : N) {
			if (j == i) continue;
			h[i][j] = IloNumVarArray(masterEnv, numNode+1);
			for (int k : N) {
				name << "h." << i << "." << j << "." << k;
				h[i][j][k] = IloNumVar(masterEnv, 0, IloInfinity, ILOINT, name.str().c_str());
				name.str("");
			}
		}
	}

	//a_i
	for (int i : freeCustomers)
	{
		name << "a." << i;
		a[i] = IloNumVar(masterEnv, 0, IloInfinity, ILOINT, name.str().c_str());
		name.str("");
	}

	//z_i
	for (int i : C)
	{
		name << "z." << i;
		z[i] = IloNumVar(masterEnv, 0, 1, ILOINT, name.str().c_str());
		name.str("");

	}


	for (int i : freeCustomers) {
		cout << instance->time_drone[arr[i]] << endl;
	}


	// OBJ FUNCTION
	IloExpr exprSolution(masterEnv);
	for (int k : N) {
		for (int i : N) {
			for (int j : N) {
				if (j == i) continue;
				exprSolution += h[i][j][k] * instance->time_truck[arr[i]][arr[j]];
			}
		}
	}

	for (int i : freeCustomers)
	{
		exprSolution += a[i] * instance->time_drone[arr[i]];
	}


	masterModel.add(IloMinimize(masterEnv, exprSolution));
	// CONSTRAINT -------------------------------------
	for (int j : truckOnly) {
		masterModel.add(z[j] == 1);
	}
	//truck
	for (int i : C)
	{
		for (int k : C)
		{
			IloExpr condij(masterEnv);
			for (int j : N)
			{
				if (j == i) continue;
				condij += x[i][j][k] - x[j][i][k - 1];
			}
			masterModel.add(condij == 0);
			condij.end();
		}
	}

	for (int i : C) {
		for (int j : C) {
			if (i == j) continue;
			masterModel.add(x[i][j][0] == 0);
		}
	}

	for (int i : C)
	{
		masterModel.add(x[i][0][0] == 0);
	}

	/*for (int i : C)
	{
		for (int j : C)
		{
			if (i == j) continue;
			for (int k : C)
			{
				IloExpr cond1(masterEnv);
				IloExpr cond2(masterEnv);
				for (int l : N)
				{
					if (l == i) continue;
					cond1 += x[l][i][k - 1];
					
				}
				for (int l : N)
				{
					if (l == j) continue;
					cond2 += x[l][j][k];

				}
				masterModel.add(x[i][j][k] >= cond1 + cond2 - 1);
				
			}
			
		}
	}*/

	IloExpr sums(masterEnv);
	for (int j : C) {
		sums += x[0][j][0];
	}
	masterModel.add(sums == 1);
	sums.end();


	IloExpr sume(masterEnv);
	for (int i : C) {
		for (int k : C) {
			sume += x[i][0][k];

		}
	}
	masterModel.add(sume == 1);
	sume.end();


	for (int k : C) {
		for (int i : C) {
			masterModel.add(x[0][i][k] == 0);
		}
	}



	for (int j : C)
	{
		IloExpr sumj(masterEnv);
		for (int i : N) {
			if (i == j) continue;
			for (int k : N) {
				sumj += x[i][j][k];
			}
		}
		masterModel.add(sumj - z[j] == 0);
		sumj.end();
	}



	for (int i : C)
	{
		IloExpr sumi(masterEnv);
		for (int j : N) {
			if (i == j) continue;
			for (int k : N) {
				sumi += x[i][j][k];
			}
		}
		masterModel.add(sumi - z[i] == 0);
		sumi.end();
	}


	for (int k : N)
	{
		IloExpr condik(masterEnv);
		for (int i : C)
		{
			condik += z[i];
		}

		IloExpr condiij(masterEnv);
		for (int i : N)
		{
			for (int j : N)
			{
				if (i == j) continue;
				condiij += x[i][j][k];
			}
		}

		masterModel.add((condik - k) * 1.0 / (numNode - 1) <= condiij);
		condik.end();
		condiij.end();
	}




	for (int i : N)
	{
		for (int j : C)
		{
			if (i == j) continue;
			for (int k : N)
			{
				masterModel.add(x[i][j][k] <= h[i][j][k]);
				masterModel.add(h[i][j][k] <= x[i][j][k] * (numNode - k));
			}
		}
	}

	for (int i : C)
	{
		for (int k : N)
		{
			masterModel.add(h[i][0][k] == 0);
		}
	}

	/*for (int i : C)
	{
		IloExpr condik(masterEnv);
		for (int k : N)
		{
			condik += s[i][k];
		}
		masterModel.add(condik == z[i]);
		condik.end();
	}*/


	/*for (int k : N)
	{
		for (int i : C)
		{
			IloExpr coni(masterEnv);
			for (int j : N)
			{
				if (j == i) continue;
				coni += x[j][i][k];
			}
			masterModel.add(coni == s[i][k]);
		}
	}

	for (int i : C)
	{
		for (int j : C)
		{
			if (i == j) continue;
			for (int k : C)
			{
				masterModel.add(x[i][j][k] <= s[i][k - 1]);
				masterModel.add(x[i][j][k] <= s[j][k]);
				masterModel.add(x[i][j][k] >= s[i][k - 1] + s[j][k] - 1);
			}
		}
	}*/

	/*for (int k : C)
	{
		IloExpr condi(masterEnv);
		IloExpr condi1(masterEnv);
		for (int i : C)
		{
			condi += s[i][k];
		}
		for (int i : C)
		{
			condi1 += s[i][k - 1];
		}
		masterModel.add(condi <= condi1);
	}*/
	IloExpr numEd(masterEnv);
	for (int i : C)
	{
		numEd += z[i];
	}
	IloExpr condi1(masterEnv);
	for (int i : N)
	{

		for (int j : C)
		{
			if (i == j) continue;
			condi1 += h[i][j][0];
		}
	}
	masterModel.add(condi1 == numEd);
	condi1.end();
	numEd.end();


	for (int k : C)
	{
		IloExpr condi1(masterEnv);
		IloExpr condi2(masterEnv);
		IloExpr condi3(masterEnv);
		IloExpr numEd(masterEnv);
		for (int i : C)
		{
			numEd += z[i];
		}
		
		for (int i : N)
		{
			for (int j : C)
			{
				if (i == j) continue;
				condi1 += h[i][j][k];
			}
		}

		for (int i : N)
		{
			for (int j : C)
			{
				if (i == j) continue;
				condi3 += h[i][j][k - 1];
			}
		}
		/*for (int i : C)
		{
			for (int v = 0; v < k; v++)
			{
					condi2 += s[i][v];
			}
		}*/
		/*for (int i : C)
		{
			for (int j : N)
			{
				if (i == j) continue;
				for (int v = 0; v < k; v++)
				{
					condi2 += x[j][i][v];
				}
			}
		}*/
		masterModel.add(condi1 >= condi3 - 1);
		condi1.end();
		condi2.end();
		condi3.end();
		numEd.end();
	}

	
	//drone

	/*for (int i : freeCustomers)
	{
		masterModel.add(z[i] == 0);
	}*/
	for (int j : freeCustomers)
	{

		masterModel.add(1 - z[j] <= a[j]);
		masterModel.add(a[j] <= (1 + j * 1.0 / numUAV) * (1 - z[j]));
	}

	for (int j : freeCustomers)
	{
		IloExpr condij(masterEnv);
		for (int i = 1; i <= j; i++)
		{
			condij += z[i];
		}
		masterModel.add((j - condij) * 1.0 / numUAV - (j * 1.0 / numUAV) * z[j] <= a[j]);

		condij.end();
	}


	masterCplex.extract(masterModel); // <<<< IMPORTANT
	cout << "Done create init MasterProblem\n";
}

void Solver::dispay_solution()
{


	unordered_map<int, int> edges;
	////    unordered_map<int,double> start;

	//    cout << "-----------x_ij-----------" << endl;
	//    for (int i : Scenes) {
	//        for(int j : Scenes){
	//            if (i == j) continue;
	//            if (cplex.getValue(y[i][j]) > 0.1){
	//                edges[i] = j;
	////                start[j] = cplex.getValue(t[i][j]);
	////                if (i == numSin)
	////                    cout << "x_" << "N" << "_" << j  << " ";
	////                else if (j == numSin)
	////                    cout << "x_" << i << "_" << "N"  << " ";
	////                else
	////                    cout << "x_" << i << "_" << j  << " ";
	//            }
	//        }
	////        cout << endl;
	//    }

	//    vector<vector<int>> tours;
	//    while (!edges.empty()) {
	//        int start = edges.begin()->first;
	//        int current = -1;
	//        vector<int> tour = {start};
	//        current = start;
	//        do {
	//            int next = edges[current];
	//            tour.push_back(next);
	//            edges.erase(current);
	//            current = next;
	//        } while (start != current);
	//        tours.push_back(tour);
	//    }

	//    for (auto tour : tours){
	//        cout << "tour: ";
	//        Utils::print_vec(tour);
	//    }








	cout << "Truck value:" << endl;;
	for (int i : N) {
		for (int j : N) {
			if (i == j) continue;
			for (int k : N) {
				if (masterCplex.getValue(x[i][j][k]) != 0) {
					cout << arr[i] << " " << arr[j] << " " << k << ",";
					cout << "h[" << arr[i] << "][" << arr[j] << "][" << k << "] = " << masterCplex.getValue(h[i][j][k]) << endl;
					cout << "tt[" << arr[i] << "][" << arr[j] << "] = " << instance->time_truck[arr[i]][arr[j]] << endl;
				}
			}

		}
	}
	cout << endl;



	/*cout << "Y value:  " << endl;
	for (int i : freeCustomers0) {
		for (int j : freeCustomers0) {
			if (i == j) continue;
			if (masterCplex.getValue(y[i][j]) == 1 && j != 0) {
				cout << i << " " << j << " " << masterCplex.getValue(y[i][j]) << ",";
				cout << "drone_time[" << j << "] = " << instance->energyModel->flightTime[j] << endl;
			}
		}
	}*/
	cout << "Drone value:  " << endl;
	for (int i : C)
	{
		if (masterCplex.getValue(z[i]) == 0)
		{
			cout << arr[i] << " " << masterCplex.getValue(a[i]) << ",";
			cout << "drone_time[" << arr[i] << "] = " << instance->time_drone[arr[i]] << endl;
		}
	}


	cout << endl;
	cout << masterCplex.getObjValue() << endl;

}

void Solver::write_output()
{

	std::ofstream ocsv;
	ocsv.open("CPLEX_output_summary.csv", std::ios_base::app);
	ocsv << instance->instanceName << ","
		<< /*fixed << std::setprecision(5) <<*/ masterCplex.getObjValue() << ","
		<< masterCplex.getBestObjValue() << ","
		<< fixed << std::setprecision(2)
		<< masterCplex.getMIPRelativeGap() * 100 << ","
		<< masterCplex.getStatus() << ","
		<< runtime << "\n"
		<< std::flush;
	ocsv.close();

	std::ofstream ocsv1;
	ocsv1.open("CPLEX_output_specific.txt", std::ios_base::app);
	ocsv1 << instance->instanceName << endl;
	ocsv1 << "Truck tour: " << endl;
	int max;
	vector<int> trace;
	int traceOr;
	int posi = 1;
	for (int j : C)
	{
		if (masterCplex.getValue(x[0][j][0]) != 0)
		{
			max = masterCplex.getValue(h[0][j][0]);
			traceOr = j;
			break;
		}
	}
	trace.resize(max+1, 0);
	trace[0] = traceOr;
	ocsv1 << "0 -> ";
	while (posi <= max) {
		for (int j : N) {
			if (trace[posi-1] == j) continue;
			if (masterCplex.getValue(x[trace[posi-1]][j][posi]) != 0) {
				posi++;
				trace[posi-1] = j;	
				//ocsv1 << trace << " -> ";
			}
		}
		
	}
	for (int i = 0; i <= max; i++)
	{
		if (i < max) ocsv1 << arr[trace[i]] << " -> ";
		else ocsv1 << arr[trace[i]];
	}
	ocsv1 << endl;
	ocsv1 << "Drone value:  " << endl;
	for (int i : C)
	{
		if (masterCplex.getValue(z[i]) == 0)
		{
			ocsv1 << arr[i] << " " << masterCplex.getValue(a[i]) << ",";
			ocsv1 << "drone_time[" << arr[i] << "] = " << instance->time_drone[arr[i]] << endl;
		}
	}
		/* masterCplex.getObjValue() << ","
		<< masterCplex.getBestObjValue() << ","
		<< fixed << std::setprecision(2)
		<< masterCplex.getMIPRelativeGap() * 100 << ","
		<< masterCplex.getStatus() << ","
		<< runtime << "\n"
		<< std::flush;*/
	ocsv1.close();
}