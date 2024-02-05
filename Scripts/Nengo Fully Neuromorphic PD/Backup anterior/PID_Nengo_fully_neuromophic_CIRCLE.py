
# Omar Garcia
# Diego Chavez
# New Mexico State University

import nengo
from quadcopterROS import Quadcopter
import gain_sets
import numpy as np


class Model(object):
    def __init__(self, target_func):
        gain_matrix_position, gain_matrix_virtual, gain_matrix_proportionals, gain_matrix_derivatives, task_to_rotor = gain_sets.get_gain_matrices(gain_set='omar')
        self.model = nengo.Network(label='PID', seed=13)

        with self.model:
            self.copter_node = Quadcopter(target_func=target_func)

            # Obtain data from class Quadcopter
            statesNode = nengo.Node(self.copter_node, size_in=4, size_out=20)  # Node from class Quadcopter
            statesEnsemble = nengo.Ensemble(n_neurons=100, dimensions=20, neuron_type=nengo.Direct())
            nengo.Connection(statesNode, statesEnsemble, synapse=None)

            # PID
            #PIDpos = nengo.Ensemble(n_neurons=250, dimensions=3, radius=0.2, neuron_type=nengo.Direct())
            PIDpos = nengo.Ensemble(n_neurons=600, dimensions=3, radius=1)
            nengo.Connection(statesEnsemble[0:7], PIDpos, transform=gain_matrix_position, synapse=None)  # Connecting Position Errors and Linear Velocity errors

            # Orientation Error
            taueOri = 0.002  # 0.00001
            # 0.0005 - 8/10, 0.005 - 6/10
            #eOri = nengo.Ensemble(n_neurons=800, dimensions=3, radius=0.8, neuron_type=nengo.Direct())
            eOri = nengo.Ensemble(n_neurons=1800, dimensions=3, radius=0.1)
            nengo.Connection(PIDpos[0:2], eOri[0:2], transform=1, synapse=taueOri)
            nengo.Connection(statesEnsemble[6:8], eOri[0:2],transform=-1, synapse=taueOri)
            nengo.Connection(statesEnsemble[12], eOri[2], transform=1, synapse=taueOri)  # Aqui tenia -1

            # Angular Velocity Inertial
            #angleVelInerDesired = nengo.Ensemble(n_neurons=500, dimensions=3, radius=0.2, neuron_type=nengo.Direct())
            angleVelInerDesired = nengo.Ensemble(n_neurons=800, dimensions=3, radius=0.4)
            nengo.Connection(eOri, angleVelInerDesired, transform=gain_matrix_virtual, synapse=0.001)

            ###################################################################################################################
            # Transforming the Desired Angular velocities from Inertial Frame to Body Frame
            #pqrDesired = nengo.Ensemble(n_neurons=1, dimensions=3, neuron_type=nengo.Direct())
            pqrDesired = nengo.Ensemble(n_neurons=600, dimensions=3, radius=0.2)



            relay = nengo.Ensemble(n_neurons=500, dimensions=5, radius=0.4, neuron_type=nengo.Direct())
            nengo.Connection(statesEnsemble[6], relay[0], synapse=None)  # Phi
            nengo.Connection(statesEnsemble[7], relay[1], synapse=None)  # Theta
            nengo.Connection(angleVelInerDesired[0], relay[2], synapse=None)  # Psi dot desired
            nengo.Connection(angleVelInerDesired[1], relay[3], synapse=None)  # Theta dot desired
            nengo.Connection(angleVelInerDesired[2], relay[4], synapse=None)  # Phi dot desired

            '''def p(x):
                return -np.sin(x[0])*x[1] + x[2]
            def q(x):
                return np.cos(x[0])*np.sin(x[1])*x[3] + np.cos(x[1])*x[2]
            def r(x):
                return np.cos(x[0])*np.cos(x[1])*x[2] - np.sin(x[0])*x[3]'''

            def p(x):
                return -np.sin(x[1]) * x[2] + x[4]

            def q(x):
                return np.cos(x[1]) * np.sin(x[0]) * x[2] + np.cos(x[0]) * x[3]

            def r(x):
                return np.cos(x[1]) * np.cos(x[0]) * x[2] - np.sin(x[0]) * x[3]

            nengo.Connection(relay, pqrDesired[0], synapse=0.01, function=p)
            nengo.Connection(relay, pqrDesired[1], synapse=0.01, function=q)
            nengo.Connection(relay, pqrDesired[2], synapse=0.01, function=r)

            synAngVelBody = 0.000
            #eAngVelBody = nengo.Ensemble(n_neurons=1, dimensions=3, neuron_type=nengo.Direct())
            eAngVelBody = nengo.Ensemble(n_neurons=1200, dimensions=3, radius=0.2)


            nengo.Connection(statesEnsemble[9:11], eAngVelBody[0:2], transform=1, synapse=synAngVelBody)
            nengo.Connection(pqrDesired[0:2], eAngVelBody[0:2], transform=-1, synapse=synAngVelBody)
            nengo.Connection(statesEnsemble[11], eAngVelBody[2], transform=-1, synapse=synAngVelBody)
            nengo.Connection(pqrDesired[2], eAngVelBody[2], synapse=synAngVelBody)


            derErrorp = nengo.Ensemble(n_neurons=300, dimensions=1, radius=2)
            derErrorq = nengo.Ensemble(n_neurons=300, dimensions=1, radius=2)
            derErrorr = nengo.Ensemble(n_neurons=300, dimensions=1, radius=2)

            synDer = 0.016
            synDerDelayed = 0.016 * 2
            derTransf = 1 / (synDerDelayed - synDer)
            nengo.Connection(eAngVelBody[0], derErrorp, synapse=synDer, transform=derTransf)
            nengo.Connection(eAngVelBody[0], derErrorp, synapse=synDerDelayed, transform=-derTransf)

            nengo.Connection(eAngVelBody[1], derErrorq, synapse=synDer, transform=derTransf)
            nengo.Connection(eAngVelBody[1], derErrorq, synapse=synDerDelayed, transform=-derTransf)

            nengo.Connection(eAngVelBody[2], derErrorr, synapse=synDer, transform=derTransf)
            nengo.Connection(eAngVelBody[2], derErrorr, synapse=synDerDelayed, transform=-derTransf)

            adjustedDerivative = nengo.Ensemble(n_neurons=1, dimensions=3, neuron_type=nengo.Direct())
            nengo.Connection(derErrorp, adjustedDerivative[0], synapse=None, transform=1)  # 0.08)
            nengo.Connection(derErrorq, adjustedDerivative[1], synapse=None, transform=1)  # 0.08)
            nengo.Connection(derErrorr, adjustedDerivative[2], synapse=None, transform=1)

            synPID = None
            # controlSignals = nengo.Ensemble(n_neurons=400, dimensions=4, radius = 0.5)
            controlSignals = nengo.Ensemble(n_neurons=1, dimensions=4, neuron_type=nengo.Direct())
            nengo.Connection(PIDpos[2], controlSignals[0], synapse=synPID)
            nengo.Connection(eAngVelBody, controlSignals[1:4], synapse=synPID, transform=gain_matrix_proportionals)
            nengo.Connection(adjustedDerivative, controlSignals[1:4], synapse=synPID, transform=gain_matrix_derivatives)

            motors = nengo.Ensemble(n_neurons=1, dimensions=4, radius=0.5, neuron_type=nengo.Direct())
            nengo.Connection(controlSignals, motors, transform=task_to_rotor)
            nengo.Connection(motors, statesNode, synapse=None)

            #########################################################################################


            # PROBES
            self.probeStatesNode = nengo.Probe(statesNode)
            self.probeStatesEnsemble = nengo.Probe(statesEnsemble, synapse=0.1)

            #self.probe_integral_ez = nengo.Probe(integral_ez, synapse = 0.1)
            self.probe_PIDpos = nengo.Probe(PIDpos, synapse=0.5)
            self.probe_eOri = nengo.Probe(eOri, synapse=0.5)
            self.probe_angleVelInerDes = nengo.Probe(angleVelInerDesired, synapse=0.5)

            #self.probe_denormalized_int_ez = nengo.Probe(denormalizedIntegralEz, synapse=0.1)
            self.probe_pqrDesired = nengo.Probe(pqrDesired, synapse=0.5)

            #self.probeInputIntegralEz = nengo.Probe(statesEnsemble[-1], synapse=0.1) # El ultimo elemento del Ensemble es la integral del error en z que fue guardada en el step anterior
            self.probe_eAngVelBody = nengo.Probe(eAngVelBody, synapse=0.5)
            self.probeDerErrorp = nengo.Probe(derErrorp, synapse= 0.1)
            self.probeDerErrorq = nengo.Probe(derErrorq, synapse= 0.1)
            self.probeDerErrorr = nengo.Probe(derErrorr, synapse= 0.1)
            self.probe_adjustedDer = nengo.Probe(adjustedDerivative, synapse=0.1)
            self.probeControlSignals = nengo.Probe(controlSignals, synapse=0.1)
            self.probeMotors = nengo.Probe(motors, synapse=0.1)



    def get_model(self):
        return self.model
    def get_copter(self):
        return self.copter_node
    def get_probe(self):
        return self.probeStatesNode, self.probeStatesEnsemble, self.probe_eOri, self.probe_angleVelInerDes, self.probe_pqrDesired, self.probe_PIDpos, self.probe_eAngVelBody, self.probeDerErrorp, self.probeDerErrorq, self.probeDerErrorr, self.probe_adjustedDer, self.probeControlSignals, self.probeMotors #, self.probe_integral_ez, self.probe_PIDpos, self.probe_eOri, self.probe_angleVelIner

