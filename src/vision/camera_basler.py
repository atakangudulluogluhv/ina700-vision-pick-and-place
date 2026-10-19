"""Basler industrial camera source, driven through pypylon.

The grab strategy is LatestImageOnly on purpose: the pipeline must always act
on what the workspace looks like now. With the default strategy the frames
queue up and the robot would be sent the position an object had several
seconds ago.

pypylon is imported inside the constructor rather than at module level so that
the rest of the project -- and the file based rehearsal at home -- still runs
on a machine where pypylon is not installed.
"""

from src.core.interfaces import CameraSource


class BaslerCamera(CameraSource):
    """First (or serial-matched) Basler device, delivered as BGR frames."""

    def __init__(self, grab_timeout_ms: int = 5000, serial_number: str = "",
                 auto_exposure: bool = True) -> None:
        try:
            from pypylon import pylon
        except ImportError as error:
            raise RuntimeError(
                "pypylon is not installed. Install it, or run with "
                "--camera file to use a recorded image instead."
            ) from error

        self._pylon = pylon
        self._grab_timeout_ms = int(grab_timeout_ms)
        self._device_model_name = "unknown"

        device = self._create_device(serial_number)
        self._camera = pylon.InstantCamera(device)
        self._camera.Open()
        self._device_model_name = self._camera.GetDeviceInfo().GetModelName()

        if auto_exposure:
            self._camera.ExposureAuto.SetValue("Continuous")

        # Bayer or mono sensor output has to be converted before OpenCV can use
        # it; BGR8 is what every cv2 call in this project expects.
        self._converter = pylon.ImageFormatConverter()
        self._converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        self._converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

        self._camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

    def _create_device(self, serial_number: str):
        """Pick the requested camera, or the only one that is attached."""
        transport_layer_factory = self._pylon.TlFactory.GetInstance()
        devices = transport_layer_factory.EnumerateDevices()
        if not devices:
            raise RuntimeError(
                "no Basler camera found. Check the cable, and for a GigE camera "
                "check that its IP is in the same subnet as this machine."
            )

        if serial_number:
            for device in devices:
                if device.GetSerialNumber() == serial_number:
                    return transport_layer_factory.CreateDevice(device)
            found = ", ".join(device.GetSerialNumber() for device in devices)
            raise RuntimeError(
                "camera with serial " + serial_number + " not found. Attached: " + found
            )
        return transport_layer_factory.CreateDevice(devices[0])

    @property
    def device_model_name(self) -> str:
        """Model name of the attached camera, logged to close open point A14."""
        return self._device_model_name

    def describe_device(self) -> str:
        """Full identification of the camera, printed once at startup."""
        info = self._camera.GetDeviceInfo()
        return (
            "model=" + info.GetModelName()
            + " serial=" + info.GetSerialNumber()
            + " interface=" + info.GetDeviceClass()
        )

    def read(self):
        """Return the newest BGR frame, or None if this grab failed."""
        if not self._camera.IsGrabbing():
            return None

        grab_result = self._camera.RetrieveResult(
            self._grab_timeout_ms, self._pylon.TimeoutHandling_ThrowException
        )
        try:
            if not grab_result.GrabSucceeded():
                return None
            return self._converter.Convert(grab_result).GetArray()
        finally:
            grab_result.Release()

    def release(self) -> None:
        if self._camera.IsGrabbing():
            self._camera.StopGrabbing()
        if self._camera.IsOpen():
            self._camera.Close()

    def __str__(self) -> str:
        return "BaslerCamera(" + self._device_model_name + ")"
