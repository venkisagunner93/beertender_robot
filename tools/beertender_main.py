#!/usr/bin/env python3

import argparse
import os
import hashlib

from os import system
from os.path import expanduser

HOME = expanduser('~')
WS = HOME + '/beertender_robot'
DOCKERFILES = WS + '/dockerfiles'

CREATE2_BASE_IMAGE_DOCKERFILE = 'Dockerfile.create2-base'
CREATE2_BEERTENDER_ROBOT_IMAGE_DOCKERFILE = 'Dockerfile.create2-beertender-robot'

CREATE2_BASE_IMAGE_DOCKERFILE_PATH = DOCKERFILES + \
    '/' + CREATE2_BASE_IMAGE_DOCKERFILE
CREATE2_BEERTENDER_ROBOT_IMAGE_DOCKERFILE_PATH = DOCKERFILES + \
    '/' + CREATE2_BEERTENDER_ROBOT_IMAGE_DOCKERFILE

SDK_BASE_IMAGE_DOCKERFILE = 'Dockerfile.sdk-base'
SDK_BEERTENDER_ROBOT_IMAGE_DOCKERFILE = 'Dockerfile.sdk-beertender-robot'

SDK_BASE_IMAGE_DOCKERFILE_PATH = DOCKERFILES + \
    '/' + SDK_BASE_IMAGE_DOCKERFILE
SDK_BEERTENDER_ROBOT_IMAGE_DOCKERFILE_PATH = DOCKERFILES + \
    '/' + SDK_BEERTENDER_ROBOT_IMAGE_DOCKERFILE

LIVE_IMAGE_NAME = 'create2-beertender-robot-live'
SDK_IMAGE_NAME = 'create2-beertender-robot-sdk'
PORT = '5000'


def build_base_image(is_robot=True):
    if is_robot:
        system('DOCKER_BUILDKIT=0 docker build -f ' +
               CREATE2_BASE_IMAGE_DOCKERFILE_PATH + ' -t create2-base:latest .')
    else:
        system('DOCKER_BUILDKIT=0 docker build -f ' +
               SDK_BASE_IMAGE_DOCKERFILE_PATH + ' -t sdk-base:latest .')


def build_beertender_robot_image(is_robot=True):
    temp_ws = '/tmp/' + hashlib.md5(os.urandom(12)).hexdigest()
    system('mkdir -p ' + temp_ws)
    system('cp -r ' + WS + '/src ' + temp_ws)
    if is_robot:
        system('cp ' + CREATE2_BEERTENDER_ROBOT_IMAGE_DOCKERFILE_PATH + ' ' + temp_ws)
        system('cd ' + temp_ws +
               ' && DOCKER_BUILDKIT=0 docker build -f ' + CREATE2_BEERTENDER_ROBOT_IMAGE_DOCKERFILE + ' -t create2-beertender-robot:latest .')
    else:
        system('cp ' + SDK_BEERTENDER_ROBOT_IMAGE_DOCKERFILE_PATH + ' ' + temp_ws)
        system('cd ' + temp_ws +
               ' && DOCKER_BUILDKIT=0 docker build -f ' + SDK_BEERTENDER_ROBOT_IMAGE_DOCKERFILE_PATH + ' -t sdk-beertender-robot:latest .')
    system('rm -rf ' + temp_ws)


def create_remote_image_name(host_ip, port, image_name):
    return host_ip + ':' + port + '/' + image_name


def push_images(host_ip):
    system('docker tag create2-beertender-robot:latest ' +
           create_remote_image_name(host_ip, PORT, LIVE_IMAGE_NAME))
    system('docker push ' + create_remote_image_name(host_ip, PORT, LIVE_IMAGE_NAME))


def pull_containers_remotely(host_ip):
    system('ssh robot@' + host_ip + ' docker pull ' +
           create_remote_image_name(host_ip, PORT, LIVE_IMAGE_NAME))


def restart_docker_containers(host_ip):
    pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Beertender robot sw dev/deployment toolkit')
    parser.add_argument(
        '-b', '--build', help='builds both base image and beertender robot image', action='store_true')
    parser.add_argument(
        '-p', '--push', help='push latest build to the beertender robot', action='store', type=str)
    parser.add_argument(
        '-s', '--sdk', help='builds both base image and beertender robot image for sdk', action='store_true')

    args = parser.parse_args()

    if args.push and args.sdk:
        print("Cannot build and push SDK image to create2 robot")

    if args.sdk:
        build_base_image(is_robot=False)
        build_beertender_robot_image(is_robot=False)
    else:
        if args.build:
            build_base_image(is_robot=True)
            build_beertender_robot_image(is_robot=True)
        elif args.push:
            build_base_image(is_robot=True)
            build_beertender_robot_image(is_robot=True)
            push_images(args.push)
            pull_containers_remotely(args.push)
            # restart_docker_containers(args.push)
