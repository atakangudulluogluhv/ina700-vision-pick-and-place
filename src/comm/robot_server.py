"""TCP server the ABB robot connects to as a client.

Role split comes straight from the assignment: the Pi is the server, the robot
is the client (trap T8), which is why the RAPID side calls
RobotAsClientConnect from ServerCommuncatio.sys. Consequences of that module,
which we are not allowed to edit (trap T18):

  * The server must already be listening before the robot program is started.
  * There is no framing. SocketReceive \\Str reads whatever arrived, so the
    exchange has to stay a strict request/response lockstep -- one message out,
    one answer back -- and no newline is appended.
  * A RAPID string holds 80 characters and truncates silently beyond that
    (trap T16), so every outgoing message is checked before it is sent.

Encodings follow the course's reference socket example: UTF-8 out, latin-1 in.
"""

import socket

MESSAGE_ID_MIN = 1


class RobotConnectionError(RuntimeError):
    """Raised when the robot is not connected or the link has dropped."""


class MessageTooLongError(ValueError):
    """Raised when a message would be truncated by RAPID's string limit."""


class RobotServer:
    """Listens for the robot client and exchanges one message at a time."""

    def __init__(
        self,
        host: str,
        port: int,
        accept_timeout_sec: float,
        ack_timeout_sec: float,
        send_encoding: str = "UTF-8",
        receive_encoding: str = "latin-1",
        receive_buffer_bytes: int = 4094,
        max_message_length: int = 80,
    ) -> None:
        self._host = host
        self._port = int(port)
        self._accept_timeout_sec = float(accept_timeout_sec)
        self._ack_timeout_sec = float(ack_timeout_sec)
        self._send_encoding = send_encoding
        self._receive_encoding = receive_encoding
        self._receive_buffer_bytes = int(receive_buffer_bytes)
        self._max_message_length = int(max_message_length)

        self._server_socket = None
        self._client_socket = None
        self._client_address = None

    @property
    def is_connected(self) -> bool:
        return self._client_socket is not None

    @property
    def client_address(self):
        return self._client_address

    def start(self) -> None:
        """Bind and listen. Must happen before the robot program is started."""
        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Without SO_REUSEADDR a restart within the TIME_WAIT window fails with
        # "address already in use", which costs minutes during a 3-hour exam.
        self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self._server_socket.bind((self._host, self._port))
        except OSError as error:
            self._server_socket.close()
            self._server_socket = None
            raise RobotConnectionError(
                "cannot bind to " + self._host + ":" + str(self._port)
                + ". Check the static IP of this machine (" + str(error) + ")"
            ) from error
        self._server_socket.listen(1)

    def wait_for_robot(self) -> str:
        """Block until the robot client connects; return its address."""
        if self._server_socket is None:
            raise RobotConnectionError("start() must be called before wait_for_robot()")

        self._server_socket.settimeout(self._accept_timeout_sec)
        try:
            client_socket, client_address = self._server_socket.accept()
        except socket.timeout as error:
            raise RobotConnectionError(
                "no robot connected within " + str(self._accept_timeout_sec)
                + " s. Is the RAPID program running, and is the port open in the firewall?"
            ) from error

        client_socket.settimeout(self._ack_timeout_sec)
        self._client_socket = client_socket
        self._client_address = client_address[0]
        return self._client_address

    def send(self, message: str) -> None:
        """Send one message to the robot, after validating it for RAPID."""
        self._validate_message(message)
        if self._client_socket is None:
            raise RobotConnectionError("no robot connected")
        try:
            self._client_socket.sendall(message.encode(self._send_encoding))
        except OSError as error:
            raise RobotConnectionError("send failed: " + str(error)) from error

    def receive(self) -> str:
        """Block until the robot answers; return its message as a string."""
        if self._client_socket is None:
            raise RobotConnectionError("no robot connected")
        try:
            payload = self._client_socket.recv(self._receive_buffer_bytes)
        except socket.timeout as error:
            raise RobotConnectionError(
                "no answer from the robot within " + str(self._ack_timeout_sec) + " s"
            ) from error
        except OSError as error:
            raise RobotConnectionError("receive failed: " + str(error)) from error

        if not payload:
            self._client_socket = None
            raise RobotConnectionError("the robot closed the connection")
        return payload.decode(self._receive_encoding).strip()

    def send_and_receive(self, message: str) -> str:
        """One lockstep round trip: send a request, wait for the answer."""
        self.send(message)
        return self.receive()

    def _validate_message(self, message: str) -> None:
        """Reject anything RAPID would truncate or fail to represent."""
        if len(message) > self._max_message_length:
            raise MessageTooLongError(
                "message is " + str(len(message)) + " characters, RAPID truncates above "
                + str(self._max_message_length) + ": " + message
            )
        if not message.isascii():
            raise MessageTooLongError("message contains non-ASCII characters: " + message)

    def drop_client(self) -> None:
        """Forget the robot but keep listening, so it can reconnect.

        After a lost or late answer the lockstep is out of step; starting a
        fresh connection is the only way to be sure both sides agree again.
        """
        if self._client_socket is not None:
            try:
                self._client_socket.close()
            except OSError:
                pass
        self._client_socket = None
        self._client_address = None

    def close(self) -> None:
        """Close the client and the listening socket, in that order."""
        for open_socket in (self._client_socket, self._server_socket):
            if open_socket is not None:
                try:
                    open_socket.close()
                except OSError:
                    pass
        self._client_socket = None
        self._server_socket = None

    def __enter__(self) -> "RobotServer":
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def __str__(self) -> str:
        state = "connected to " + str(self._client_address) if self.is_connected else "waiting"
        return "RobotServer(" + self._host + ":" + str(self._port) + ", " + state + ")"


class MessageFormatter:
    """Renders a detection into the exact string the RAPID routine parses.

    The layout is a template from settings.yaml rather than hard-coded, because
    the RAPID routine already exists and we do not yet know which layout it
    expects. Adapting in the lab is then a one-line YAML edit, not a code edit.
    """

    def __init__(self, template: str) -> None:
        self._template = template

    def format(
        self,
        message_id: int,
        shape: str,
        color: str = "unknown",
        x_mm: float = 0.0,
        y_mm: float = 0.0,
        z_mm: float = 0.0,
        rz_deg: float = 0.0,
    ) -> str:
        """Fill the template; unused placeholders simply stay unused."""
        return self._template.format(
            id=message_id,
            shape=shape,
            shape_upper=shape.upper(),
            color=color,
            color_upper=color.upper(),
            x_mm=round(x_mm, 1),
            y_mm=round(y_mm, 1),
            z_mm=round(z_mm, 1),
            rz_deg=round(rz_deg, 1),
        )

    def __str__(self) -> str:
        return "MessageFormatter(" + repr(self._template) + ")"
