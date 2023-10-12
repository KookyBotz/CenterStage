public class Actuator {
  private Map<String, HardwareDevice> devices = new ConcurrentHashMap<>();
  private double position;
  private AsymmetricalMotionProfile profile;
  private PIDController controller;
  private ElapsedTime timer;

  public Actuator(HardwareDevice... devices) {
    for (HardwareDevice device : devices) {
      this.devices.put(device.getName(), device);
    }
  }

  public void read() {
    for (HardwareDevice device : devices) {
      // Different checks for different object methods
      if (device instanceOf AnalogServo) {
        this.position = device.getPosition();
      } else if (device instanceOf DcMotor) {
        this.position = device.getPosition();
      }
    }
  }

  public HardwareDevice getDevice(String deviceName) {
    return this.devices.get(deviceName);
  }

  public List<HardwareDevice> getDevices() {
    return new ArrayList<>(devices.values());
  }

  public void setMotionProfile(AsymmetricalMotionProfile profile) {
    this.profile = profile;
  }

  public void setMotionConstraints(ProfileConstraints constraints) {
    this.constraints = constraints;
  }

  public void setPIDController(PIDFController controller) {
    this.controller = controller;
  }

  public double getPosition() {
    return position;
  }
}
