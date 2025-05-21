####################################################
# DVrouter.py
# Name:
# HUID:
#####################################################

from router import Router
from packet import Packet

INFINITY = 16  # Maximum cost (simulated)

class DVrouter(Router):
    """Distance Vector routing protocol implementation."""

    def __init__(self, addr, heartbeat_time):
        super().__init__(addr)
        self.heartbeat_time = heartbeat_time
        self.last_time = 0

        # Distance vector table: {dest_addr: (cost, next_hop_port)}
        self.distance_vector = {addr: (0, None)}  # Cost to self is 0

        # Neighbor's advertised distance vectors: {neighbor_addr: {dest: cost}}
        self.neighbor_vectors = {}

        # Mapping ports to neighbors: {port: neighbor_addr}
        self.port_to_neighbor = {}

    def handle_packet(self, port, packet):
        """Process incoming packet."""
        if packet.is_traceroute:
            dst = packet.dst_addr
            if dst in self.distance_vector:
                out_port = self.distance_vector[dst][1]
                if out_port is not None:
                    self.send(out_port, packet)
        else:
            # Routing packet: handle received distance vector
            sender = packet.src_addr
            vector = packet.costs
            self.neighbor_vectors[sender] = vector  # Update neighbor's vector

            updated = False
            link_cost = self.get_link(port)[1]

            for dest, neighbor_cost in vector.items():
                if dest == self.addr:
                    continue  # Skip self

                total_cost = min(INFINITY, neighbor_cost + link_cost)

                if dest not in self.distance_vector:
                    self.distance_vector[dest] = (total_cost, port)
                    updated = True
                else:
                    current_cost, current_port = self.distance_vector[dest]
                    # Nếu route mới tốt hơn, hoặc nếu route cũ đi qua neighbor này mà cost thay đổi
                    if (total_cost < current_cost) or (current_port == port and total_cost != current_cost):
                        self.distance_vector[dest] = (total_cost, port)
                        updated = True

            # Nếu có cập nhật thì broadcast vector mới cho neighbors
            if updated:
                self.broadcast_vector()

    def handle_new_link(self, port, endpoint, cost):
        """Handle new link."""
        # C
