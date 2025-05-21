####################################################
# DVrouter.py
# Name:
# HUID:
#####################################################

from router import Router
from packet import Packet  # Giả định bạn có packet class như thế này

INFINITY = 16  # max cost (giả lập)

class DVrouter(Router):
    """Distance vector routing protocol implementation."""

    def __init__(self, addr, heartbeat_time):
        Router.__init__(self, addr)
        self.heartbeat_time = heartbeat_time
        self.last_time = 0

        # Distance vector: {destination: (cost, next_hop_port)}
        self.distance_vector = {addr: (0, None)}  # Khoảng cách đến chính nó là 0
        # Neighbor distances: {neighbor_addr: {dest: cost}}
        self.neighbor_vectors = {}

    def handle_packet(self, port, packet):
        """Process incoming packet."""
        if packet.is_traceroute:
            dst = packet.dst_addr
            if dst in self.distance_vector:
                out_port = self.distance_vector[dst][1]
                if out_port is not None:
                    self.send(out_port, packet)
        else:
            # Routing packet (distance vector)
            sender = packet.src_addr
            vector = packet.costs

            updated = False
            self.neighbor_vectors[sender] = vector

            for dest, cost in vector.items():
                link_cost = self.get_link(port)[1]
                total_cost = min(INFINITY, cost + link_cost)

                if dest == self.addr:
                    continue

                # Nếu chưa có route hoặc tìm thấy route tốt hơn
                if (dest not in self.distance_vector) or (total_cost < self.distance_vector[dest][0]):
                    self.distance_vector[dest] = (total_cost, port)
                    updated = True

            # Nếu có cập nhật — broadcast vector mới cho các neighbors
            if updated:
                self.broadcast_vector()

    def handle_new_link(self, port, endpoint, cost):
        """Handle new link."""
        # Thêm neighbor mới
        self.neighbor_vectors[endpoint] = {}
        # Cập nhật distance vector đến neighbor
        self.distance_vector[endpoint] = (cost, port)
        self.broadcast_vector()

    def handle_remove_link(self, port):
        """Handle removed link."""
        removed_neighbor = None
        for neighbor, vector in self.neighbor_vectors.items():
            if self.distance_vector.get(neighbor, (None, None))[1] == port:
                removed_neighbor = neighbor
                break

        if removed_neighbor:
            del self.neighbor_vectors[removed_neighbor]

        # Loại bỏ tất cả route đi qua port này
        to_delete = []
        for dest, (c, p) in self.distance_vector.items():
            if p == port:
                to_delete.append(dest)

        for dest in to_delete:
            del self.distance_vector[dest]

        self.broadcast_vector()

    def handle_time(self, time_ms):
        """Handle current time."""
        if time_ms - self.last_time >= self.heartbeat_time:
            self.last_time = time_ms
            self.broadcast_vector()

    def broadcast_vector(self):
        """Gửi distance vector đến các neighbors."""
        for port in self.get_ports():
            endpoint, _ = self.get_link(port)
            # Tạo bản sao distance vector
            vector_to_send = {}
            for dest, (cost, next_port) in self.distance_vector.items():
                # Split horizon with poison reverse
                if next_port == port:
                    vector_to_send[dest] = INFINITY
                else:
                    vector_to_send[dest] = cost

            packet = Packet(self.addr, endpoint, False, vector_to_send)
            self.send(port, packet)

    def __repr__(self):
        return f"DVrouter(addr={self.addr}, dv={self.distance_vector})"
