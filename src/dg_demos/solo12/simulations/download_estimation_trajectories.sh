#! /bin/bash

download_trajectory()
{
    download_fodler=$1
    mkdir -p $download_fodler
    cd $download_fodler
    wget --no-check-certificate https://memmo-data.laas.fr/media/artifacts/Legged\ Robot\ Estimation/ReferenceTrajectories/$download_fodler/joint_positions.dat
    wget --no-check-certificate https://memmo-data.laas.fr/media/artifacts/Legged\ Robot\ Estimation/ReferenceTrajectories/$download_fodler/joint_velocities.dat
    wget --no-check-certificate https://memmo-data.laas.fr/media/artifacts/Legged\ Robot\ Estimation/ReferenceTrajectories/$download_fodler/joint_torques.dat
    cd -
}

download_trajectory stamping
download_trajectory com_oscillation
