import os
import traci
import numpy as np
import subprocess
import sys

from icv_model import ICVModel
from chv_model import CHVModel
from dbscan_cluster import perform_dbscan_clustering


def run():
    sumo_cmd = [
        "sumo-gui",
        "-c",
        "model.sumocfg",
        "--step-length",
        "0.1",
       
    ]

    # start sumo
    sumo_proc = subprocess.Popen(sumo_cmd, stdout=sys.stdout, stderr=sys.stderr)

    # wait for sumo to start
    traci.init(63781)

    # create ICV
    icv_id = "ICV"
    icv_model = ICVModel()

    # create CHVs
    chv_ids = ["CHV" + str(i) for i in range(1, 6)]
    chv_models = [CHVModel() for _ in chv_ids]

    # create DBSCAN cluster
    dbscan_cluster = perform_dbscan_clustering(5, 10)

    # simulation loop
    try:
        step = 0
        while traci.simulation.getMinExpectedNumber() > 0:
            # advance simulation
            traci.simulationStep()

            # update models
            icv_model.update(traci.vehicle.getSubscriptionResults(icv_id))
            chv_models = [model.update(traci.vehicle.getSubscriptionResults(chv_id)) for model, chv_id in zip(chv_models, chv_ids)]

            # get relevant vehicle data for clustering
            chv_data = [
                {"id": chv_id, "position": chv_model.position, "velocity": chv_model.velocity}
                for chv_id, chv_model in zip(chv_ids, chv_models)
            ]

            # cluster CHVs
            chv_clusters = dbscan_cluster.cluster(chv_data)

            # control ICV
            if step == 0:
                icv_model.init_control(traci.vehicle.getSubscriptionResults(icv_id), chv_clusters)

            icv_acc = icv_model.control(traci.vehicle.getSubscriptionResults(icv_id), chv_clusters)
            traci.vehicle.setAccel(icv_id, icv_acc)

            # control CHVs
            for chv_model, chv_cluster in zip(chv_models, chv_clusters):
                if chv_cluster is None:
                    continue

                lead_id = chv_cluster["lead_id"]
                lead_velocity = traci.vehicle.getSubscriptionResults(lead_id).getSpeed()
                chv_acc = chv_model.control(traci.vehicle.getSubscriptionResults(chv_model.id), traci.vehicle.getSubscriptionResults(lead_id), lead_velocity)
                traci.vehicle.setAccel(chv_model.id, chv_acc)

            step += 1

    finally:
        traci.close()
        sumo_proc.terminate()
        sumo_proc.wait()


if __name__ == "__main__":
    run()
