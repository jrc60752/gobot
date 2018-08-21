package m4metro

import (
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"sync"

	multierror "github.com/hashicorp/go-multierror"
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/drivers/i2c"
	"gobot.io/x/gobot/drivers/spi"
	"gobot.io/x/gobot/sysfs"
)

type pwmPinData struct {
	channel int
	path    string
}

const pwmDefaultPeriod = 500000

// Adaptor is the gobot.Adaptor representation for the M4 Metro
type Adaptor struct {
	name               string
	digitalPins        []*sysfs.DigitalPin
	pwmPins            map[string]*sysfs.PWMPin
	i2cBuses           map[int]i2c.I2cDevice
	usrLed             string
	analogPath         string
	pinMap             map[string]int
	pwmPinMap          map[string]pwmPinData
	analogPinMap       map[string]string
	mutex              *sync.Mutex
	findPin            func(pinPath string) (string, error)
	spiDefaultBus      int
	spiDefaultChip     int
	spiBuses           [2]spi.Connection
	spiDefaultMode     int
	spiDefaultMaxSpeed int64
}

// NewAdaptor returns a new M4 Metro Adaptor
func NewAdaptor() *Adaptor {
	m := &Adaptor{
		name:         gobot.DefaultName("M4Metro"),
		digitalPins:  make([]*sysfs.DigitalPin, 120),
		pwmPins:      make(map[string]*sysfs.PWMPin),
		i2cBuses:     make(map[int]i2c.I2cDevice),
		mutex:        &sync.Mutex{},
		pinMap:       m4mPinMap,
		pwmPinMap:    m4mPwmPinMap,
		analogPinMap: m4mAnalogPinMap,
		findPin: func(pinPath string) (string, error) {
			files, err := filepath.Glob(pinPath)
			return files[0], err
		},
	}

	m.setPaths()
	return m
}

func (m *Adaptor) setPaths() {
	//TODO: What is this stuff??
	m.usrLed = "/sys/class/leds/beaglebone:green:"
	m.analogPath = "/sys/bus/iio/devices/iio:device0"

	m.spiDefaultBus = 0
	m.spiDefaultMode = 0
	m.spiDefaultMaxSpeed = 500000
}

// Name returns the Adaptor name
func (m *Adaptor) Name() string { return m.name }

// SetName sets the Adaptor name
func (m *Adaptor) SetName(n string) { m.name = n }

// Connect initializes the pwm and analog dts.
func (m *Adaptor) Connect() error {
	return nil
}

// Finalize releases all i2c devices and exported analog, digital, pwm pins.
func (m *Adaptor) Finalize() (err error) {
	m.mutex.Lock()
	defer m.mutex.Unlock()

	for _, pin := range m.digitalPins {
		if pin != nil {
			if e := pin.Unexport(); e != nil {
				err = multierror.Append(err, e)
			}
		}
	}
	for _, pin := range m.pwmPins {
		if pin != nil {
			if e := pin.Unexport(); e != nil {
				err = multierror.Append(err, e)
			}
		}
	}
	for _, bus := range m.i2cBuses {
		if bus != nil {
			if e := bus.Close(); e != nil {
				err = multierror.Append(err, e)
			}
		}
	}
	for _, bus := range m.spiBuses {
		if bus != nil {
			if e := bus.Close(); e != nil {
				err = multierror.Append(err, e)
			}
		}
	}
	return
}

// PwmWrite writes the 0-254 value to the specified pin
//TODO - verify values, etc for htis function
func (m *Adaptor) PwmWrite(pin string, val byte) (err error) {
	pwmPin, err := m.PWMPin(pin)
	if err != nil {
		return
	}
	period, err := pwmPin.Period()
	if err != nil {
		return err
	}
	duty := gobot.FromScale(float64(val), 0, 255.0)
	return pwmPin.SetDutyCycle(uint32(float64(period) * duty))
}

// ServoWrite writes a servo signal to the specified pin
func (m *Adaptor) ServoWrite(pin string, angle byte) (err error) {
	pwmPin, err := m.PWMPin(pin)
	if err != nil {
		return
	}

	// TODO: take into account the actual period setting, not just assume default
	const minDuty = 100 * 0.0005 * pwmDefaultPeriod
	const maxDuty = 100 * 0.0020 * pwmDefaultPeriod
	duty := uint32(gobot.ToScale(gobot.FromScale(float64(angle), 0, 180), minDuty, maxDuty))
	return pwmPin.SetDutyCycle(duty)
}

// DigitalRead returns a digital value from specified pin
func (m *Adaptor) DigitalRead(pin string) (val int, err error) {
	sysfsPin, err := m.DigitalPin(pin, sysfs.IN)
	if err != nil {
		return
	}
	return sysfsPin.Read()
}

// DigitalWrite writes a digital value to specified pin.
// valid usr pin values are usr0, usr1, usr2 and usr3
func (m *Adaptor) DigitalWrite(pin string, val byte) (err error) {
	if strings.Contains(pin, "usr") {
		fi, e := sysfs.OpenFile(m.usrLed+pin+"/brightness", os.O_WRONLY|os.O_APPEND, 0666)
		defer fi.Close()
		if e != nil {
			return e
		}
		_, err = fi.WriteString(strconv.Itoa(int(val)))
		return err
	}
	sysfsPin, err := m.DigitalPin(pin, sysfs.OUT)
	if err != nil {
		return err
	}
	return sysfsPin.Write(int(val))
}

// DigitalPin retrieves digital pin value by name
func (m *Adaptor) DigitalPin(pin string, dir string) (sysfsPin sysfs.DigitalPinner, err error) {
	m.mutex.Lock()
	defer m.mutex.Unlock()

	i, err := m.translatePin(pin)
	if err != nil {
		return
	}
	if m.digitalPins[i] == nil {
		m.digitalPins[i] = sysfs.NewDigitalPin(i)
		if err = muxPin(pin, "gpio"); err != nil {
			return
		}

		err := m.digitalPins[i].Export()
		if err != nil {
			return nil, err
		}
	}
	if err = m.digitalPins[i].Direction(dir); err != nil {
		return
	}
	return m.digitalPins[i], nil
}

// PWMPin returns matched pwmPin for specified pin number
func (m *Adaptor) PWMPin(pin string) (sysfsPin sysfs.PWMPinner, err error) {
	m.mutex.Lock()
	defer m.mutex.Unlock()

	pinInfo, err := m.translatePwmPin(pin)
	if err != nil {
		return nil, err
	}

	if m.pwmPins[pin] == nil {
		newPin := sysfs.NewPWMPin(pinInfo.channel)
		if err = muxPin(pin, "pwm"); err != nil {
			return
		}

		if newPin.Path, err = m.findPin(pinInfo.path); err != nil {
			return
		}
		if err = newPin.Export(); err != nil {
			return
		}
		if err = newPin.SetPeriod(pwmDefaultPeriod); err != nil {
			return
		}
		if err = newPin.Enable(true); err != nil {
			return
		}
		m.pwmPins[pin] = newPin
	}

	sysfsPin = m.pwmPins[pin]

	return
}

// AnalogRead returns an analog value from specified pin
func (m *Adaptor) AnalogRead(pin string) (val int, err error) {
	analogPin, err := m.translateAnalogPin(pin)
	if err != nil {
		return
	}
	fi, err := sysfs.OpenFile(fmt.Sprintf("%v/%v", m.analogPath, analogPin), os.O_RDONLY, 0644)
	defer fi.Close()

	if err != nil {
		return
	}

	var buf = make([]byte, 1024)
	_, err = fi.Read(buf)
	if err != nil {
		return
	}

	val, _ = strconv.Atoi(strings.Split(string(buf), "\n")[0])
	return
}

// GetConnection returns a connection to a device on a specified bus.
// Valid bus number is either 0 or 2 which corresponds to /dev/i2c-0 or /dev/i2c-2.
func (m *Adaptor) GetConnection(address int, bus int) (connection i2c.Connection, err error) {
	m.mutex.Lock()
	defer m.mutex.Unlock()

	if (bus != 0) && (bus != 2) {
		return nil, fmt.Errorf("Bus number %d out of range", bus)
	}
	if m.i2cBuses[bus] == nil {
		m.i2cBuses[bus], err = sysfs.NewI2cDevice(fmt.Sprintf("/dev/i2c-%d", bus))
	}
	return i2c.NewConnection(m.i2cBuses[bus], address), err
}

// GetDefaultBus returns the default i2c bus for this platform
func (m *Adaptor) GetDefaultBus() int {
	return 2
}

// GetSpiConnection returns an spi connection to a device on a specified bus.
// Valid bus number is [0..1] which corresponds to /dev/spidev0.0 through /dev/spidev0.1.
func (m *Adaptor) GetSpiConnection(busNum, chipNum, mode, bits int, maxSpeed int64) (connection spi.Connection, err error) {
	m.mutex.Lock()
	defer m.mutex.Unlock()

	if (busNum < 0) || (busNum > 1) {
		return nil, fmt.Errorf("Bus number %d out of range", busNum)
	}

	if m.spiBuses[busNum] == nil {
		m.spiBuses[busNum], err = spi.GetSpiConnection(busNum, chipNum, mode, bits, maxSpeed)
	}

	return m.spiBuses[busNum], err
}

// GetSpiDefaultBus returns the default spi bus for this platform.
func (m *Adaptor) GetSpiDefaultBus() int {
	return m.spiDefaultBus
}

// GetSpiDefaultChip returns the default spi chip for this platform.
func (m *Adaptor) GetSpiDefaultChip() int {
	return m.spiDefaultChip
}

// GetSpiDefaultMode returns the default spi mode for this platform.
func (m *Adaptor) GetSpiDefaultMode() int {
	return m.spiDefaultMode
}

// GetSpiDefaultBits returns the default spi number of bits for this platform.
func (m *Adaptor) GetSpiDefaultBits() int {
	return 8
}

// GetSpiDefaultMaxSpeed returns the default spi bus for this platform.
func (m *Adaptor) GetSpiDefaultMaxSpeed() int64 {
	return m.spiDefaultMaxSpeed
}

// translatePin converts digital pin name to pin position
func (m *Adaptor) translatePin(pin string) (value int, err error) {
	if val, ok := m.pinMap[pin]; ok {
		value = val
	} else {
		err = errors.New("Not a valid pin")
	}
	return
}

func (m *Adaptor) translatePwmPin(pin string) (p pwmPinData, err error) {
	if val, ok := m.pwmPinMap[pin]; ok {
		p = val
	} else {
		err = errors.New("Not a valid PWM pin")
	}
	return
}

// translateAnalogPin converts analog pin name to pin position
func (m *Adaptor) translateAnalogPin(pin string) (value string, err error) {
	if val, ok := m.analogPinMap[pin]; ok {
		value = val
	} else {
		err = errors.New("Not a valid analog pin")
	}
	return
}

func muxPin(pin, cmd string) error {
	path := fmt.Sprintf("/sys/devices/platform/ocp/ocp:%s_pinmux/state", pin)
	fi, e := sysfs.OpenFile(path, os.O_WRONLY, 0666)
	defer fi.Close()
	if e != nil {
		return e
	}
	_, e = fi.WriteString(cmd)
	return e
}
