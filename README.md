## Install pip packages:
```
python3 -m pip install grpcio grpcio-tools grpclib protobuf gcloud
```

## Install C# packages:
```
dotnet add package Grpc.Tools --global
```

## Install go packages:
```
go install google.golang.org/grpc/cmd/protoc-gen-go-grpc@latest \
    google.golang.org/protobuf/cmd/protoc-gen-go@latest
    github.com/grpc-ecosystem/grpc-gateway/v2/protoc-gen-grpc-gateway@latest \
    github.com/grpc-ecosystem/grpc-gateway/v2/protoc-gen-openapiv2@latest \
    google.golang.org/protobuf/cmd/protoc-gen-go@latest \
    google.golang.org/grpc/cmd/protoc-gen-go-grpc@latest
```

## Build protos:

### Build python protos:
```
python3 -m grpc_tools.protoc -I=./protos --python_out=./interfases --grpc_python_out=./interfases ./protos/midterm.proto
```

### Build csharp protos:
```
protoc -I=./protos --csharp_out=./csharp_client --grpc_csharp_out=./csharp_client ./protos/midterm.proto
```

### Build go protos:
```
protoc -I ./protos --go_out ./gateway/service --go_opt paths=source_relative \
    --grpc-gateway_out ./gateway/service --grpc-gateway_opt paths=source_relative \
    --go-grpc_out ./gateway/service --go-grpc_opt paths=source_relative \
    ./protos/midterm.proto
```

## Build
```
colcon build --symlink-install
source install/setup.sh
```

## Run color detector
```
ros2 run interfases color_detect.py
```

## Run grcp wrapper
```
ros2 run interfases wrapper.py
```

## Run csharp grpc client
```
cd src/interfases/csharp_client
dotnet run
```

## Run go grpc gateway
```
cd src/interfases/gateway
go run main/gateway.go
```