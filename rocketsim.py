#!/usr/bin/python

from scitools.std import *

def main():
  sr = 200                #Sampling Rate
  dt = 1.0/sr
  rocket = Rocket(dt)
  t = [0]       #time in seconds
  lastBurn = 0
  while rocket.height[rocket.index] > 0.0 and rocket.index * dt < 1000:
    t.append(t[rocket.index]+dt)
    rocket.step()
    if(rocket.height[rocket.index] <= 100 and not rocket.firing):
      rocket.firing = True
    if(rocket.height[rocket.index] <= 1500 and not rocket.parachute):
      rocket.parachute = True
    if(rocket.height[rocket.index] <= 3):
      rocket.dm = -6
      lastBurn = rocket.index
      break
  while rocket.height[rocket.index] > 0.0:
    t.append(t[rocket.index]+dt)
    rocket.step()
    if lastBurn + 60 == rocket.index:
      rocket.dm = -7
    if lastBurn + 65 == rocket.index:
      rocket.firing = False



  plot(t, rocket.height)
  xlabel('time')
  ylabel('height')
  title("Height")
  axis([600, 800, 0, 1300])
  figure()
  plot(t, rocket.speed)
  xlabel('time')
  ylabel('speed')
  title("Speed")
  axis([600, 800, -12, 8])
  #figure()
  #plot(t, rocket.acceleration)
  #xlabel('time')
  #ylabel('acceleration')
  #title("Acceleration")
  #figure()
  #plot(t, rocket.f__all, 'k')
  #hold('on')
  #plot(t, rocket.f__gravity, 'g')
  #plot(t, rocket.f__drag, 'b')
  #plot(t, rocket.f__thrust, 'r')
  #xlabel('time')
  #ylabel('Force')
  #title("Gravity")

  raw_input()

class Environment():

  earth_radius  = 6357500             # Radius of the earth [m]
  earth_mass    = 5.9721986 * 10 **24 # Mass of the earth [kg]
  grav_const    = 6.67 * 10 ** -11    # Newton-Gravitational-Constant [m**3/(kg*s**2)]
  pAir0         = 101325              # sea level standard air pressure [Pa]
  TAir0         = 288.15              # sea level standard temperature [K]
  dT            = 0.0065               # Temperature lapse rate [K/m]
  mAirMol       = 0.0289644           # molar mass of dry air [kg/mol]
  gasConst      = 8.31447             # universal gas constant [J/(mol*K]


  def gravity(self, height):
    return self.earth_mass * self.grav_const / ((self.earth_radius + height) ** 2)

  def temperature(self, height):
    return self.TAir0 - (self.dT * height)

  def air_pressure(self, height):
    if height < 15000:
      return self.pAir0 * ((1-(self.dT * height)/self.TAir0)**((self.gravity(0)*self.mAirMol)/(self.gasConst * self.dT)))
    else:
      return 0

  def air_density(self, height):
    return self.air_pressure(height)/(self.gasConst*self.temperature(height))

class Rocket():
  mRocket     = 900.0   #131000.0  # mass of the rocket without fuel [kg]
  mFuel       = [230] #[2169000.0] # inital mass of the fuel [kg]
  dm          = -4    #-13360.0  # flow of mass from the rocket [kg/s]
  Cd          = 0.5       # Drag Coefficient
  A_front     = 3.0       # frontal surface area [m**2]
  Cd_para     = 0.5
  A_para      = 10.0
  specImpulse = 200  #263.0     # specific Impulse by weight [s] look at wikipedia
  height      = [20000.0]
  speed       = [-800.0]
  acceleration= [0.0]
  burnRate   = [0.0]
  f__all      = [0.0]
  f__drag     = [0.0]
  f__gravity  = [0.0]
  f__thrust   = [0.0]
  firing = False
  parachute = False

  def __init__(self, dt):
    self.env    = Environment()
    self.dt = dt
    self.index = 0

  def calc_burn_rate(self, index):
    if self.firing:
      newBurn = -1 * ((self.burnRate[index-1] * 0.05) + (self.dm - self.burnRate[index-1]))
      if newBurn<=0:
        newBurn = 0
      elif newBurn >= self.dm:
        newBurn = self.dm
      return newBurn
    else:
      return 0.0

  def calc_fuel(self, index):
    if self.firing:
      newFuel = self.mFuel[index-1] + self.burnRate[index] * self.dt
      if newFuel <= 0:
        self.firing = False
        newFuel = 0
      return newFuel
    else:
      return self.mFuel[index-1]

  def f_gravity(self, index):
    grav = -1 * self.env.gravity(self.height[index])
    return grav * ( self.mRocket + self.mFuel[index])

  def f_drag(self, index):
    if self.parachute and self.speed[index] < 0:
      return 0.5 * self.env.air_density(self.height[index]) * self.Cd * self.A_front * self.speed[index] ** 2 + 0.5 * self.env.air_density(self.height[index]) * self.Cd_para * self.A_para * self.speed[index] ** 2
    if self.speed[index] > 0:
      return -0.5 * self.env.air_density(self.height[index]) * self.Cd * self.A_front * self.speed[index] ** 2
    else:
      return 0.5 * self.env.air_density(self.height[index]) * self.Cd * self.A_front * self.speed[index] ** 2

  def f_thrust(self, index):
    if self.firing == True:
      return self.specImpulse * -1 * self.env.gravity(self.height[index]) * self.burnRate[index]
    else:
      return 0

  def f(self, index):
    return self.f_thrust(index) + self.f_drag(index) + self.f_gravity(index)

  def calc_acceleration(self, index):
    return self.f(index-1)/(self.mFuel[index-1]+self.mRocket)

  def calc_speed(self, index):
    return self.speed[index-1] + self.acceleration[index] * self.dt

  def calc_height(self, index):
    new_height = self.height[index-1] + self.speed[index] * self.dt
    if new_height < 0:
      self.speed.pop()
      self.speed.append(0.0)
      return 0.0
    else:
      return self.height[index-1] + self.speed[index] * self.dt

  def step(self):
    self.index  = self.index + 1
    self.burnRate.append(self.calc_burn_rate(self.index))
    self.mFuel.append(self.calc_fuel(self.index))
    self.acceleration.append(self.calc_acceleration(self.index))
    self.speed.append(self.calc_speed(self.index))
    self.height.append(self.calc_height(self.index))
    self.f__all.append(self.f(self.index))
    self.f__gravity.append(self.f_gravity(self.index))
    self.f__drag.append(self.f_drag(self.index))
    self.f__thrust.append(self.f_thrust(self.index))

main()
