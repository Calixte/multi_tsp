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

import solver.constraints.Propagator;
import solver.constraints.PropagatorPriority;
import solver.exception.ContradictionException;
import solver.variables.EventType;
import solver.variables.IntVar;
import util.ESat;
import util.graphOperations.connectivity.ConnectivityFinder;
import util.objects.graphs.UndirectedGraph;
import util.objects.setDataStructures.SetType;
import util.tools.ArrayUtils;

/**
 * Propagator ensuring that the graph induced by a set of successor variables
 * has nbCC connected components (CC)
 * @author Jean-Guillaume Fages
 */
public class PropNbCC extends Propagator<IntVar<?>> {

	//***********************************************************************************
	// VARIABLES
	//***********************************************************************************

	private static final long serialVersionUID = -8024292091025873113L;
	int n;
	IntVar<?>[] succs;
	IntVar<?> nb;
	UndirectedGraph g;
	ConnectivityFinder ccf;


	//***********************************************************************************
	// CONSTRUCTORS
	//***********************************************************************************

	/**
	 * Creates a propagator ensuring that the graph induced by succs has nbCC connected components
	 * @param succs	a set of integer variables
	 * @param nbCC
	 */
	public PropNbCC(IntVar<?>[] succs, IntVar<?> nbCC) {
		super(ArrayUtils.append(succs, new IntVar[]{nbCC}), PropagatorPriority.LINEAR,false);
		this.n = succs.length;
		this.succs = succs;
		this.nb = nbCC;
		this.g = new UndirectedGraph(n, SetType.BITSET,true);
		this.ccf = new ConnectivityFinder(g);
	}

	//***********************************************************************************
	// METHODS
	//***********************************************************************************

	/**
	 * Awakening conditions (any variable domain modification)
	 * @param	vIdx variable which has changed
	 * @return	a mask of modifications (int_all_mask)
	 */
	protected int getPropagationConditions(int vIdx) {
		return EventType.INT_ALL_MASK();
	}

	/**
	 * Main filtering algorithm:
	 * TODO
	 * 2) Implement filtering procedure of PropKCC (see isEntail)
	 * @param evtmask					(useless for the exam)
	 * @throws ContradictionException	(when a fail occurs)
	 */
	public void propagate(int evtmask) throws ContradictionException {
		for(int i=0;i<n;i++){
			g.getNeighborsOf(i).clear();
		}
		for(int i=0;i<n;i++){
			for(int j=succs[i].getLB(); j<=succs[i].getUB(); j=succs[i].nextValue(j)){
				g.addEdge(i,j);
			}
		}
		ccf.findAllCC();
		nb.updateLowerBound(ccf.getNBCC(), aCause);
		for(int i=0;i<n;i++){
			g.getNeighborsOf(i).clear();
		}
		for(int i=0;i<n;i++){
			if(succs[i].instantiated()){
				g.addEdge(i,succs[i].getValue());
			}
		}
		ccf.findAllCC();
		nb.updateUpperBound(ccf.getNBCC(), aCause);
	}

	/**
	 * Incremental propagation (nothing particular)
	 * @param idxVarInProp
	 * @param mask
	 * @throws ContradictionException
	 */
	public void propagate(int idxVarInProp, int mask) throws ContradictionException {
		forcePropagate(EventType.FULL_PROPAGATION);
	}

	/**
	 * Entailment checker
	 * @return ESat.FALSE if the propagator is violated
	 * 		   ESat.TRUE if the propagator is satisfied for sure, no matter what happens next
	 * 		   ESat.UNDEFINED otherwise
	 */
	public ESat isEntailed() {
		// estimates the minimum number of cc of the solution by
		// computing cc of the graph induced by variable domains
		for(int i=0;i<n;i++){
			g.getNeighborsOf(i).clear();
		}
		for(int i=0;i<n;i++){
			for(int j=succs[i].getLB(); j<=succs[i].getUB(); j=succs[i].nextValue(j)){
				g.addEdge(i,j);
			}
		}
		ccf.findAllCC();
		if(nb.getUB()<ccf.getNBCC()){
			// too many CC
			return ESat.FALSE;
		}
		// estimates the maximum number of cc of the solution by
		// computing cc of the graph induced by instantiated variable values
		for(int i=0;i<n;i++){
			g.getNeighborsOf(i).clear();
		}
		for(int i=0;i<n;i++){
			if(succs[i].instantiated()){
				g.addEdge(i,succs[i].getValue());
			}
		}
		ccf.findAllCC();
		if(nb.getLB()>ccf.getNBCC()){
			// not enough CC
			return ESat.FALSE;
		}
		// if all variables are instantiated then this propagator is entailed
		if(isCompletelyInstantiated()){
			return ESat.TRUE;
		}else{
			return ESat.UNDEFINED;
		}
	}
}
