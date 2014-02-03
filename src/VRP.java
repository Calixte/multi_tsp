/**
 *  Copyright (c) 1999-2011, Ecole des Mines de Nantes
 *  All rights reserved.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Ecole des Mines de Nantes nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE REGENTS AND CONTRIBUTORS BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import samples.AbstractProblem;
import samples.graph.input.TSP_Utils;
import solver.ResolutionPolicy;
import solver.Solver;
import solver.constraints.Constraint;
import solver.constraints.ICF;
import solver.constraints.LogicalConstraintFactory;
import solver.constraints.Propagator;
import solver.search.loop.monitors.IMonitorSolution;
import solver.search.loop.monitors.SMF;
import solver.search.strategy.ISF;
import solver.search.strategy.strategy.AbstractStrategy;
import solver.search.strategy.strategy.StrategiesSequencer;
import solver.variables.IntVar;
import solver.variables.VF;
import solver.variables.VariableFactory;

/**
 * Discrete Optimization Exam: a Vehicle Routing Problem
 *
 * You have to:
 * 1) Implement constraints of the model						(5 pts)
 * 2) Implement filtering procedure of PropKCC (see isEntail)	(4 pts)
 * 3) Call the prettyOut() method on every solution				(2 pts)
 * 4) Finds a search strategy which tends to					(4 pts)
 *  a. use few trucks
 *  b. use a lot of fuel
 * 5) Enrich the model with truck variables						(5 pts)
 * a. creates truck variables that indicate the truck going to each vertex
 * b. add associated constraints
 *
 * Bonus) Find a way to improve the solutions you get			(5 pts)
 *
 * @author Jean-Guillaume Fages
 */
public class VRP extends AbstractProblem {

	//***********************************************************************************
	// VARIABLES
	//***********************************************************************************

	// constants
	final static int NB_MAX_TRUCKS = 5;
	final static int FUEL_COST = 1;
	final static int TRUCK_COST = 500;
	final static long TIME_LIMIT = 60000;// in ms
	final static String INSTANCE_DIR = "inst/";

	// instance variables
	int n;
	int[][] DIST_MATRIX;
	String instanceName = "burma14";

	// cp variables
	IntVar<?>[] succs;
	IntVar<?>[] sucFuel;
	IntVar<?>[] trucks;
	IntVar<?> fuel, nbTrucks, objective;

	// TODO 5)a. Enrich the model with truck variables

	//***********************************************************************************
	// METHODS
	//***********************************************************************************

	@Override
	public void createSolver() {
		solver = new Solver("Discrete Optimization Exam: a Vehicle Routing Problem");
		level = Level.SILENT;
	}

	@Override
	public void buildModel() {
		// load instance
		int infinity = Integer.MAX_VALUE/10;
		DIST_MATRIX = TSP_Utils.parseInstance(INSTANCE_DIR+"/"+instanceName+".tsp",infinity);
		n = DIST_MATRIX.length;

		int _max = 0;
		for(int i = 0 ; i < n ; i++) {
			int max = 0;
			for(int j = 0 ; j < n ; j++) {
				max = Math.max(i, DIST_MATRIX[i][j]);
			}
			_max = Math.max(_max, max);
		}
		
		// build variables
		succs = VF.enumeratedArray("successor", n, 0, n-1, solver);
		sucFuel = VF.boundedArray("cost of successor", n, 0, infinity, solver);
		fuel = VF.bounded("tour cost", 0, _max*n, solver);
		nbTrucks = VF.bounded("nb trucks", 1, NB_MAX_TRUCKS, solver);
		objective = VF.bounded("objective", 0, infinity, solver);
		vars5a();

		// add constraints
		cstrs1();
		cstrs5b();
	}

	@Override
	public void configureSearch() {
		pretty3();
		solver.set(search4a());
//		solver.set(search4b());
	}

	@Override
	public void solve() {
		// time limit
		SMF.limitTime(solver,TIME_LIMIT);
		// optimize
		solver.findOptimalSolution(ResolutionPolicy.MINIMIZE, objective);
		System.out.println(solver.getMeasures());
	}

	@Override
	public void prettyOut() {
		String s = "- Solution found, cost = "+objective.getValue()
				+"\n"+nbTrucks.getValue()+" trucks"
				+"\n"+fuel.getValue()+" fuel consumed";
		s+= "\nin "+(int)(solver.getMeasures().getTimeCount()/1000)+" sec";
		System.out.println(s);
	}

	public static void main(String[] args){
		new VRP().execute(args);
	}

	//***********************************************************************************
	// METHODS TO COMPLETE
	//***********************************************************************************

	/**
	 * TODO:
	 * 1) Implement constraints of the model
	 */
	@SuppressWarnings("unchecked")
	private void cstrs1() {
		solver.post(ICF.alldifferent(succs, "AC"));
		for(int i = 0 ; i < n ; i++) {
			solver.post(ICF.element(sucFuel[i], DIST_MATRIX[i], succs[i]));
		}
		int[] fuel_costs = new int[n];
		for(int i = 0 ; i < n ; i++) {
			fuel_costs[i] = FUEL_COST;
		}
		solver.post(ICF.scalar(sucFuel, fuel_costs, fuel));
		solver.post(ICF.scalar(new IntVar[]{fuel, nbTrucks}, new int[]{1, TRUCK_COST}, objective));
		Constraint<IntVar<?>, Propagator<IntVar<?>>> c = new Constraint<IntVar<?>, Propagator<IntVar<?>>>(succs, solver);
		c.addPropagators(new PropNbCC(succs, nbTrucks));
		solver.post(c);
	}

	/**
	 * TODO
	 * 3) Call the prettyOut() method on every solution
	 */
	private void pretty3() {
		solver.getSearchLoop().plugSearchMonitor(new IMonitorSolution() {

			private static final long serialVersionUID = 6846326258772435614L;

			@Override
			public void onSolution() {
				prettyOut();
			}
		});
	}

	/**
	 * TODO
	 * 4) Finds a search strategy which tends to
	 *  a. use few trucks
	 */
	public AbstractStrategy<?> search4a(){
		return new StrategiesSequencer(
				ISF.inputOrder_InDomainMin(new IntVar[]{nbTrucks}),
				ISF.random(succs,0));
	}

	/**
	 * TODO
	 * 4) Finds a search strategy which tends to
	 *  b. use a lot of fuel
	 */
	public AbstractStrategy<?> search4b(){
		return new StrategiesSequencer(
				ISF.inputOrder_InDomainMax(new IntVar[]{fuel}),
				ISF.random(succs,0));
	}

	/**
	 * TODO
	 * 5) Enrich the model with truck variables
	 * a. creates truck variables that indicate the truck going to each vertex
	 */
	private void vars5a() {
		trucks = VariableFactory.enumeratedArray("trucks", n, 0, NB_MAX_TRUCKS, solver);
		for(int i = 0 ; i < n ; i++) {
			for(int j = 0 ; j < n ; j++) {
				solver.post(LogicalConstraintFactory.ifThen(
						ICF.arithm(VariableFactory.fixed(j, solver), "=", succs[i]),
						ICF.arithm(trucks[i], "=", trucks[j])));
			}
		}
		//TODO s'assurer que deux noeuds qui ne sont pas dans le même chemin ont un numéro de truck différent
	}

	/**
	 * TODO
	 * 5) Enrich the model with truck variables
	 * b. add associated constraints
	 */
	private void cstrs5b() {
		//throw new UnsupportedOperationException("TODO 5)b.");
	}
}
