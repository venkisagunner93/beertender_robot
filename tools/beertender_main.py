#!/usr/bin/env python3

import argparse
import os
import hashlib

from os import system
from os.path import expanduser

HOME = expanduser('~')
WS = HOME + '/beertender_robot'
DOCKERFILES = WS + '/dockerfiles'

BASE_IMAGE_DOCKERFILE = 'Dockerfile.create2-base'
BEERTENDER_ROBOT_IMAGE_DOCKERFILE = 'Dockerfile.create2-beertender-robot'

BASE_IMAGE_DOCKERFILE_PATH = DOCKERFILES + '/' + BASE_IMAGE_DOCKERFILE
BEERTENDER_ROBOT_IMAGE_DOCKERFILE_PATH = DOCKERFILES + \
    '/' + BEERTENDER_ROBOT_IMAGE_DOCKERFILE


def build_base_image():
    system('DOCKER_BUILDKIT=1 docker build -f ' +
           BASE_IMAGE_DOCKERFILE_PATH + ' -t create2-base:latest .')


def build_robot_image():
    temp_ws = '/tmp/' + hashlib.md5(os.urandom(12)).hexdigest()
    system('mkdir -p ' + temp_ws)
    system('cp -r ' + WS + '/install ' + temp_ws)
    system('cp ' + BEERTENDER_ROBOT_IMAGE_DOCKERFILE_PATH + ' ' + temp_ws)
    system('cd ' + temp_ws +
           ' && DOCKER_BUILDKIT=1 docker build -f ' + BEERTENDER_ROBOT_IMAGE_DOCKERFILE + ' -t create2-beertender-robot:latest .')
    system('rm -rf ' + temp_ws)


def build_images():
    build_base_image()
    build_robot_image()


def push_images(host_ip):
    system('docker tag create2-beertender-robot:latest ' +
           host_ip + ':5000/create2-beertender-robot-live')
    system('docker push ' + host_ip + ':5000/create2-beertender-robot-live')


def restart_docker_containers(host_ip):
    pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Beertender robot sw dev/deployment toolkit')
    parser.add_argument(
        '-b', '--build', help='builds both base image and beertender robot image', action='store_true')
    parser.add_argument(
        '-p', '--push', help='push latest builds to the beertender robot', action='store', type=str)

    args = parser.parse_args()

    if args.build:
        build_images()
    elif args.push:
        build_images()
        push_images(args.push)
        restart_docker_containers(args.push)
