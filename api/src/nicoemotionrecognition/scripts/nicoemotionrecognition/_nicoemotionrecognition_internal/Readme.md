### Frame-Based Emotion Categorization

Both implemented KERAS models are based on the Face Channel of the Crosschanel CNN - more information can be found here:

```sh
Barros, P., & Wermter, S. (2016). Developing crossmodal expression recognition based on a deep neural model. Adaptive behavior, 24(5), 373-396.
http://journals.sagepub.com/doi/full/10.1177/1059712316664017
```

### Build Docker and update container registry

The necessary steps to build the Docker container and upload it to the repository are combined in the following script:

```bash
bash build_and_upload_docker.bash
```

This will log you in, generate the tag for the container with the active git branch, build the Docker image and then upload it to the repository.
