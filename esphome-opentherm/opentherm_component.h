#include "esphome.h"
#include "esphome/components/sensor/sensor.h"
#include "OpenTherm.h"
#include "opentherm_switch.h"
#include "opentherm_climate.h"
#include "opentherm_binary.h"
#include "opentherm_output.h"
#include<cmath>

// Pins to OpenTherm Adapter
int inPin = D2; 
int outPin = D1;
OpenTherm ot(inPin, outPin, false);

bool mock = true;

IRAM_ATTR void handleInterrupt() {
	ot.handleInterrupt();
}

enum Step{
  SetReadStatus,
  SetCHSetpoint,
  SetDHWSetpoint,
  ReadRelModulationLevel,
  ReadCHTemperature,
  ReadReturnTemperature,
  ReadCHPressure,
  ReadDHWTemperature,
  ReadDHWFlowRate,
  LAST
};

class OpenthermComponent: public PollingComponent {
private:
  const char *TAG = "opentherm_component";
  OpenthermFloatOutput *pid_output_;
  int step_ = 0;
  int retry_times_ = 0;
  float last_central_heating_setpoint_tmp_ = -1;
  float last_central_heating_setpoint_ = -1;
  float last_hot_water_setpoint_tmp_ = -1;
  float last_hot_water_setpoint_ = -1;

  void next_step() {
    if (++step_ >= Step::LAST) {
      step_ = 0;
    }
    retry_times_ = 0;
  }

public:
  static std::vector<OpenthermComponent*> components;
  OpenthermSwitch *thermostatSwitch = new OpenthermSwitch();
  Sensor *external_temperature_sensor = new Sensor();
  Sensor *return_temperature_sensor = new Sensor();
  Sensor *boiler_temperature = new Sensor();
  Sensor *pressure_sensor = new Sensor();
  Sensor *flow_rate_sensor = new Sensor();
  Sensor *modulation_sensor = new Sensor();
  Sensor *heating_target_temperature_sensor = new Sensor();
  OpenthermClimate *hotWaterClimate = new OpenthermClimate();
  OpenthermClimate *heatingWaterClimate = new OpenthermClimate();
  BinarySensor *flame = new OpenthermBinarySensor();
  
  OpenthermComponent(): PollingComponent(1000) {
    components.push_back(this);
  }

  static void handle_response(unsigned long response, OpenThermResponseStatus responseStatus){
    for (int i=0; i<components.size(); i++) {
      components[i]->on_opentherm_response(response, responseStatus);
    }
  }

  void set_pid_output(OpenthermFloatOutput *pid_output) { pid_output_ = pid_output; }

  unsigned long mock_response() {
    switch (step_) {
      case Step::SetReadStatus:
        return ot.buildResponse(OpenThermMessageType::READ_ACK, OpenThermMessageID::Status, B00001110);
      case Step::SetCHSetpoint:
        return ot.buildResponse(OpenThermMessageType::WRITE_ACK, OpenThermMessageID::TSet, 0);
      case Step::SetDHWSetpoint:
        return ot.buildResponse(OpenThermMessageType::WRITE_ACK, OpenThermMessageID::TdhwSet, 0);
      case Step::ReadRelModulationLevel:
        return ot.buildResponse(OpenThermMessageType::READ_ACK, OpenThermMessageID::RelModLevel, ot.temperatureToData(80.5));
      case Step::ReadCHTemperature:
        return ot.buildResponse(OpenThermMessageType::READ_ACK, OpenThermMessageID::Tboiler, ot.temperatureToData(50.5));
      case Step::ReadReturnTemperature:
        return ot.buildResponse(OpenThermMessageType::READ_ACK, OpenThermMessageID::Tret, ot.temperatureToData(41.2));
      case Step::ReadCHPressure:
        return ot.buildResponse(OpenThermMessageType::READ_ACK, OpenThermMessageID::CHPressure, ot.temperatureToData(80.5));
      case Step::ReadDHWTemperature:
        return ot.buildResponse(OpenThermMessageType::READ_ACK, OpenThermMessageID::Tdhw, ot.temperatureToData(41.5));
      case Step::ReadDHWFlowRate:
        return ot.buildResponse(OpenThermMessageType::READ_ACK, OpenThermMessageID::DHWFlowRate, ot.temperatureToData(37.83));
      default:
        return 0;
    }
  }

  void on_opentherm_response(unsigned long response, OpenThermResponseStatus responseStatus) {
    if (mock) {
      response = mock_response();
      responseStatus = OpenThermResponseStatus::SUCCESS;
    }
    if (responseStatus == OpenThermResponseStatus::SUCCESS) {
      switch (ot.getDataID(response)) {
        case OpenThermMessageID::Status:
        {
          bool is_flame_on = ot.isFlameOn(response);
          bool is_central_heating_active = ot.isCentralHeatingActive(response);
          bool is_hot_water_active = ot.isHotWaterActive(response);
          ESP_LOGD(TAG, "Central Heating: %s", ONOFF(is_central_heating_active));
          ESP_LOGD(TAG, "Hot Water: %s", ONOFF(is_hot_water_active));
          ESP_LOGD(TAG, "Flame: %s", ONOFF(is_flame_on));
          flame->publish_state(is_flame_on); 
          hotWaterClimate->action = is_hot_water_active ? ClimateAction::CLIMATE_ACTION_HEATING : ClimateAction::CLIMATE_ACTION_OFF;
          hotWaterClimate->publish_state();
          heatingWaterClimate->action = is_central_heating_active && is_flame_on ? ClimateAction::CLIMATE_ACTION_HEATING : ClimateAction::CLIMATE_ACTION_OFF;
          heatingWaterClimate->publish_state();
          break;
        }

        case OpenThermMessageID::Tboiler:
        {
          float temperature = ot.getFloat(response);
          ESP_LOGD(TAG, "Central heating temperature is %f °C", temperature);
          boiler_temperature->publish_state(temperature);
          heatingWaterClimate->current_temperature = temperature;
          hotWaterClimate->publish_state();
          break;
        }

        case OpenThermMessageID::Tret:
        {
          float temperature = ot.getFloat(response);
          ESP_LOGD(TAG, "Return temperature is %f °C", temperature);
          return_temperature_sensor->publish_state(temperature);
          break;
        }

        case OpenThermMessageID::CHPressure:
        {
          float pressure = ot.getFloat(response);
          ESP_LOGD(TAG, "Central heating pressure is %f bar", pressure);
          pressure_sensor->publish_state(pressure);
          break;
        }

        case OpenThermMessageID::RelModLevel:
        {
          float modulation = ot.getFloat(response);
          ESP_LOGD(TAG, "Relative modulation level is %f%%", modulation);
          modulation_sensor->publish_state(modulation);
          break;
        }

        case OpenThermMessageID::Tdhw:
        {
          float temperature = ot.getFloat(response);
          ESP_LOGD(TAG, "Domestic hot water temperature is %f °C", temperature);
          hotWaterClimate->current_temperature = temperature;
          hotWaterClimate->publish_state();
          break;
        }

        case OpenThermMessageID::DHWFlowRate:
        {
          float flow_rate = ot.getFloat(response);
          ESP_LOGD(TAG, "Domestic hot water flow rate is %f L/min", flow_rate);
          flow_rate_sensor->publish_state(flow_rate);
          break;   
        }

        case OpenThermMessageID::TdhwSet:
        {
          last_hot_water_setpoint_ = last_hot_water_setpoint_tmp_;
          ESP_LOGD(TAG, "Domestic hot water setpoint is %f °C", last_hot_water_setpoint_);
        }

        case OpenThermMessageID::TSet:
        {
          last_central_heating_setpoint_ = last_central_heating_setpoint_tmp_;
          ESP_LOGD(TAG, "Central heating setpoint is %f °C", last_central_heating_setpoint_);
        }
          
        default: 
          break;
      }
      next_step();
    } else {
      if (responseStatus == OpenThermResponseStatus::NONE)
        ESP_LOGD(TAG, "OpenTherm is not initialized");
      if (responseStatus == OpenThermResponseStatus::TIMEOUT)
        ESP_LOGD(TAG, "Request timeout");
      if (responseStatus == OpenThermResponseStatus::INVALID)
        ESP_LOGD(TAG, "Response invalid: 0x%08X", response);
    }
  }

  void setup() override {
    // This will be called once to set up the component
    // think of it as the setup() call in Arduino
      ESP_LOGD(TAG, "Setup");

      ot.begin(handleInterrupt, handle_response);
      thermostatSwitch->add_on_state_callback([=](bool state) -> void {
        ESP_LOGD (TAG, "termostatSwitch_on_state_callback %d", state);    
      });

      // Adjust HeatingWaterClimate depending on PID
      // heatingWaterClimate->set_supports_heat_cool_mode(this->pid_output_ != nullptr);
      heatingWaterClimate->set_supports_two_point_target_temperature(this->pid_output_ != nullptr);

      hotWaterClimate->set_temperature_settings(5, 6, 5.5);
      heatingWaterClimate->set_temperature_settings(19.5, 20.5, 20);
      hotWaterClimate->setup();
      heatingWaterClimate->setup();
      thermostatSwitch->setup();
  }

  bool setBolierStatusAsync(bool enableCentralHeating, bool enableHotWater) {
    return ot.sendRequestAync(ot.buildSetBoilerStatusRequest(enableCentralHeating, enableHotWater, false));
  }

  bool setBoilerTemperatureAsync(float temperature) {
    return ot.sendRequestAync(ot.buildSetBoilerTemperatureRequest(temperature));
  }

  bool getBoilerTemperatureAsync() {
    return ot.sendRequestAync(ot.buildGetBoilerTemperatureRequest());
  }

  bool getReturnTemperatureAsync() {
    return ot.sendRequestAync(ot.buildRequest(OpenThermRequestType::READ, OpenThermMessageID::Tret, 0));
  }
  
  bool getDomesticHotWaterTemperatureAsync() {
    return ot.sendRequestAync(ot.buildRequest(OpenThermRequestType::READ, OpenThermMessageID::Tdhw, 0));
  }

  bool setDomesticHotWaterTemperatureAsync(float temperature) {
	  unsigned int data = ot.temperatureToData(temperature);
    unsigned long request = ot.buildRequest(OpenThermRequestType::WRITE, OpenThermMessageID::TdhwSet, data);
    return ot.sendRequestAync(request);
  }

  bool getRelativeModulationLevelAsync() {
    return ot.sendRequestAync(ot.buildRequest(OpenThermRequestType::READ, OpenThermMessageID::RelModLevel, 0));
  }

  bool getPressureAsync() {
    return ot.sendRequestAync(ot.buildRequest(OpenThermRequestType::READ, OpenThermMessageID::CHPressure, 0));
  }

  bool getDomesticHotWaterFlowRateAsync() {
    return ot.sendRequestAync(ot.buildRequest(OpenThermRequestType::READ, OpenThermMessageID::DHWFlowRate, 0));
  }

  void update() override {

    ot.process();

    if (!ot.isReady()){
      ESP_LOGD(TAG, "Opentherm API is not ready");
      return;
    }

    if (retry_times_ >= 3){
      next_step();
    }

    bool enable_central_heating = heatingWaterClimate->mode == ClimateMode::CLIMATE_MODE_HEAT;
    bool enable_hot_water = hotWaterClimate->mode == ClimateMode::CLIMATE_MODE_HEAT;

    execute_step:
    bool success = false;
    retry_times_++;

    switch (step_){
      case Step::SetReadStatus:
        //Set/Read Boiler Status
        ESP_LOGD(TAG, "Setting central heating status: %s", ONOFF(enable_central_heating));
        ESP_LOGD(TAG, "Setting domestic hot water status: %s", ONOFF(enable_hot_water));
        success = setBolierStatusAsync(enable_central_heating, enable_hot_water);
        break;

      case Step::SetCHSetpoint:
        if (enable_central_heating) {
          // Set temperature depending on room thermostat
          float heating_target_temperature;
          const char* source;
          
          if (this->pid_output_ != nullptr && thermostatSwitch->state) {
            float pid_output = pid_output_->get_state();
            if (pid_output == 0.0f) {
              heating_target_temperature = 10.0f;
            }
            else {
              heating_target_temperature =  pid_output * (heatingWaterClimate->target_temperature_high - heatingWaterClimate->target_temperature_low) 
              + heatingWaterClimate->target_temperature_low;      
            }
            source = "modulating output";
          }
          else {
            heating_target_temperature = heatingWaterClimate->target_temperature;
            source = "central heating climate";
          }

          if (round(last_central_heating_setpoint_) != round(heating_target_temperature)) {
            ESP_LOGD(TAG, "Setting central heating temperature at %f °C (from %s)", heating_target_temperature, source);
            last_central_heating_setpoint_tmp_ = heating_target_temperature;
            heating_target_temperature_sensor->publish_state(heating_target_temperature);
            success = setBoilerTemperatureAsync(heating_target_temperature);
          } else {
            next_step();
            goto execute_step;
          }
        } else {
          next_step();
          goto execute_step;
        }
        break;

      case Step::SetDHWSetpoint:
        if (enable_hot_water) {
          // Set hot water temperature
          float domestic_hot_water_target_temperature = hotWaterClimate->target_temperature;
          if (round(last_hot_water_setpoint_) != round(domestic_hot_water_target_temperature)) {
            last_hot_water_setpoint_tmp_ = domestic_hot_water_target_temperature;
            ESP_LOGD(TAG, "Setting domestic hot water temperature at %f °C...", domestic_hot_water_target_temperature);
            success = setDomesticHotWaterTemperatureAsync(domestic_hot_water_target_temperature);
          } else {
            next_step();
            goto execute_step;
          }
        } else {
          next_step();
          goto execute_step;
        }
        break;

      case Step::ReadCHTemperature:
        if (enable_central_heating) {
          ESP_LOGD(TAG, "Getting central heating water temperature...");
          success = getBoilerTemperatureAsync();
        } else {
          next_step();
          goto execute_step;
        }
        break;


      case Step::ReadReturnTemperature:
        if (enable_central_heating) {
          ESP_LOGD(TAG, "Getting return temperature...");
          success = getReturnTemperatureAsync();
        } else {
          next_step();
          goto execute_step;
        }
        break;

      case Step::ReadRelModulationLevel:
        if (enable_central_heating) {
          ESP_LOGD(TAG, "Getting relative modulation level...");
          success = getRelativeModulationLevelAsync();
        } else {
          next_step();
          goto execute_step;
        }
        break;

      case Step::ReadDHWTemperature:
        if (enable_hot_water) {
          ESP_LOGD(TAG, "Getting domestic hot water temperature...");
          success = getDomesticHotWaterTemperatureAsync();
        } else {
          next_step();
          goto execute_step;
        }
        break;

      case Step::ReadDHWFlowRate:
        if (enable_hot_water) {
          ESP_LOGD(TAG, "Getting domestic hot water flow rate...");
          success = getDomesticHotWaterFlowRateAsync();
        } else {
          next_step();
          goto execute_step;
        }
        break;
      
      case Step::ReadCHPressure:
        if (enable_central_heating) {
          ESP_LOGD(TAG, "Getting central heating water pressure");
          success = getPressureAsync();
        } else {
          next_step();
          goto execute_step;
        }
        break;

      default:
        break;
    }
  }

};

std::vector<OpenthermComponent*> OpenthermComponent::components;