# dronelocate demo, containerised.
#
# The point of this image is not deployment -- it is that "which Python is
# this and does it have zenoh" stops being a question. On the machine this
# was written on, `python3` was 3.6 and unusable while the deps lived under
# 3.11; on a colleague's laptop it will be something else again. Here it is
# always 3.11 with the deps present.
#
#   docker build -t dronelocate .
#   docker run --rm -p 8080:8080 dronelocate
#   → http://localhost:8080
#
# Everything runs sim-only: ten emulated sensor nodes, no radio hardware.
FROM python:3.11-slim

# scipy needs a BLAS at runtime; nothing here needs a compiler because all
# four dependencies publish manylinux wheels.
RUN apt-get update \
 && apt-get install -y --no-install-recommends libgomp1 procps \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Dependencies first so edits to the source do not invalidate the wheel layer.
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

# The console. Zenoh's 7447 stays internal: every node in this image talks to
# the supernode over loopback, so there is nothing to publish.
EXPOSE 8080

# Fail the build rather than the demo if the pieces cannot even be imported.
RUN python -c "import numpy, scipy, zenoh, cbor2; \
    from dronelocate import proto, tdoa, geo, sigsim; \
    print('deps + package import OK')"

# run_demo.sh probes for a usable interpreter and tails the supernode log.
# --init so ctrl-c reaches it and its eleven children get cleaned up.
CMD ["./run_demo.sh"]
