# set base image (host OS)
FROM tensorflow/tensorflow:1.14.0-gpu-py3

# set the working directory in the container
WORKDIR /code

# copy the dependencies file to the working directory
COPY requirements.txt .

# install dependencies
RUN apt-get update && apt-get install -y cmake
RUN apt-get install -y git
RUN apt-get install -y libgtk2.0-dev pkg-config
RUN pip install -r requirements.txt

# copy the content of the local src directory to the working directory
COPY . .

# command to run on container start
ENTRYPOINT [ "python", "./start_server.py" ]
