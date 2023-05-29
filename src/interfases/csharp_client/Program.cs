using System;
using Grpc.AspNetCore;
using Grpc.Core;
using Midterm;

namespace Client
{
    class Program
    {
        static void Main(string[] args)
        {
            Channel channel = new Channel("localhost:50051", ChannelCredentials.Insecure);
            var client = new ColorDetector.ColorDetectorClient(channel);
            var request = new global::Google.Protobuf.WellKnownTypes.Empty();
            var reply = client.getCoordinate(request);
            Console.WriteLine("Server replied with coordinate ({0}, {1})", reply.X, reply.Y);
            channel.ShutdownAsync().Wait();
            Console.WriteLine("Press any key to exit...");
            Console.ReadKey();
        }
    }
}