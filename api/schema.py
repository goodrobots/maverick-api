from __future__ import absolute_import
import graphene
from rx.subjects import Subject

class StreamState:
    stream={} # this dict holds the current telem message data for each message type

class Subscriptions:
    stream={} # this dict holds Subject() for each message type
    
class Parameters:
    params={}
    

class ParameterBase(graphene.Interface):
    id = graphene.ID()
    
class ParameterFloat(graphene.ObjectType):
    class Meta:
        interfaces = (ParameterBase, )
    value = graphene.Float()
    
    @classmethod
    def create(cls, id, value):
        _id = 'State'
        param_float = cls(id=id, value=value)
        # update the local storage content
        Parameters.params[id] = param_float
        return param_float

class ParameterInt(graphene.ObjectType):
    class Meta:
        interfaces = (ParameterBase, )
    value = graphene.Int()
    
    @classmethod
    def create(cls, id, value):
        _id = 'State'
        param_int = cls(id=id, value=value)
        # update the local storage content
        Parameters.params[id] = param_int
        return param_int
        
class ParameterCollection(graphene.ObjectType):
    params = graphene.List(ParameterBase)


class TelemMessage(graphene.Interface):
    id = graphene.ID()
    seq = graphene.Int()
    secs = graphene.Int()
    nsecs = graphene.Int()
    frame_id = graphene.String()

### start State Message
class StateMessage(graphene.ObjectType):
    class Meta:
        interfaces = (TelemMessage, )
    connected = graphene.Boolean()
    armed = graphene.Boolean()
    guided = graphene.Boolean()
    mode = graphene.String()
    system_status = graphene.Int()
    
    @classmethod
    def create(cls, seq, secs, nsecs, frame_id, connected, armed, guided, mode, system_status):
        _id = 'State'
        state_message = cls(id=_id, seq = seq, secs = secs, nsecs = nsecs, frame_id = frame_id, 
            connected = connected, armed = armed, guided = guided, mode = mode, system_status = system_status)
        # update the local storage content
        StreamState.stream[_id] = state_message
        return state_message
    
class UpdateStateMessage(graphene.Mutation):
    class Arguments:
        id = graphene.ID()
        seq = graphene.Int()
        secs = graphene.Int()
        nsecs = graphene.Int()
        frame_id = graphene.String()
        connected = graphene.Boolean()
        armed = graphene.Boolean()
        guided = graphene.Boolean()
        mode = graphene.String()
        system_status = graphene.Int()

    ok = graphene.Boolean()
    state_message = graphene.Field(lambda: StateMessage)
    
    def mutate(self, info = None, **kwargs):
        state_message = StateMessage.create(**kwargs)
        ok = True
        # notify subscribers of an update
        Subscriptions.stream['State'].on_next(state_message) 
        return UpdateStateMessage(state_message=state_message, ok=ok)
### end State Message

### start VfrHud Message
class VfrHudMessage(graphene.ObjectType):
    class Meta:
        interfaces = (TelemMessage, )
    airspeed = graphene.Float()
    groundspeed = graphene.Float()
    heading = graphene.Int()
    throttle = graphene.Float()
    altitude = graphene.Float()
    climb = graphene.Float()
    
    @classmethod
    def create(cls, seq, secs, nsecs, frame_id, airspeed, groundspeed, heading, throttle, altitude, climb):
        _id = 'VfrHud'
        vfr_hud_message = cls(id=_id, seq = seq, secs = secs, nsecs = nsecs, frame_id = frame_id, 
            airspeed = airspeed, groundspeed = groundspeed, heading = heading, throttle = throttle, altitude = altitude, climb = climb)
        # update the local storage content
        StreamState.stream[_id] = vfr_hud_message
        return vfr_hud_message
    
class UpdateVfrHudMessage(graphene.Mutation):
    class Arguments:
        id = graphene.ID()
        seq = graphene.Int()
        secs = graphene.Int()
        nsecs = graphene.Int()
        frame_id = graphene.String()
        airspeed = graphene.Float()
        groundspeed = graphene.Float()
        heading = graphene.Int()
        throttle = graphene.Float()
        altitude = graphene.Float()
        climb = graphene.Float()

    ok = graphene.Boolean()
    vfr_hud_message = graphene.Field(lambda: VfrHudMessage)
    
    def mutate(self, info = None, **kwargs):
        vfr_hud_message = VfrHudMessage.create(**kwargs)
        ok = True
        # notify subscribers of an update
        Subscriptions.stream['VfrHud'].on_next(vfr_hud_message)
        return UpdateVfrHudMessage(vfr_hud_message=vfr_hud_message, ok=ok)
### end VfrHud Message

### start PoseStamped Message
class PoseStampedMessage(graphene.ObjectType):
    class Meta:
        interfaces = (TelemMessage, )
    pose_position_x = graphene.Float()
    pose_position_y = graphene.Float()
    pose_position_z = graphene.Float()
    pose_orientation_x = graphene.Float()
    pose_orientation_y = graphene.Float()
    pose_orientation_z = graphene.Float()
    pose_orientation_w = graphene.Float()
    
    @classmethod
    def create(cls, seq, secs, nsecs, frame_id,
               pose_position_x, pose_position_y, pose_position_z, pose_orientation_x, pose_orientation_y, pose_orientation_z, pose_orientation_w):
        _id = 'PoseStamped'
        pose_stamped_message = cls(id=_id, seq = seq, secs = secs, nsecs = nsecs, frame_id = frame_id, 
            pose_position_x = pose_position_x, pose_position_y = pose_position_y, pose_position_z = pose_position_z,
            pose_orientation_x = pose_orientation_x, pose_orientation_y = pose_orientation_y, pose_orientation_z = pose_orientation_z)
        # update the local storage content
        StreamState.stream[_id] = pose_stamped_message
        return pose_stamped_message
    
class UpdatePoseStampedMessage(graphene.Mutation):
    class Arguments:
        id = graphene.ID()
        seq = graphene.Int()
        secs = graphene.Int()
        nsecs = graphene.Int()
        frame_id = graphene.String()
        pose_position_x = graphene.Float()
        pose_position_y = graphene.Float()
        pose_position_z = graphene.Float()
        pose_orientation_x = graphene.Float()
        pose_orientation_y = graphene.Float()
        pose_orientation_z = graphene.Float()
        pose_orientation_w = graphene.Float()

    ok = graphene.Boolean()
    pose_stamped_message = graphene.Field(lambda: PoseStampedMessage)
    
    def mutate(self, info = None, **kwargs):
        pose_stamped_message = PoseStampedMessage.create(**kwargs)
        ok = True
        # notify subscribers of an update
        Subscriptions.stream['PoseStamped'].on_next(pose_stamped_message)
        return UpdatePoseStampedMessage(pose_stamped_message=pose_stamped_message, ok=ok)
### end PoseStamped Message

### start Imu Message
class ImuMessage(graphene.ObjectType):
    class Meta:
        interfaces = (TelemMessage, )
    orientation_x = graphene.Float()
    orientation_y = graphene.Float()
    orientation_z = graphene.Float()
    orientation_w = graphene.Float()
    # TODO: orientation covariance array
    angular_velocity_x = graphene.Float()
    angular_velocity_y = graphene.Float()
    angular_velocity_z = graphene.Float()
    # TODO: angular velocity covariance array
    linear_acceleration_x = graphene.Float()
    linear_acceleration_y = graphene.Float()
    linear_acceleration_z = graphene.Float()
    # TODO: linear acceleration covariance array
    # TODO: calculate and provide roll, pitch, yaw euler angles
    
    @classmethod
    def create(cls, seq, secs, nsecs, frame_id, orientation_x, orientation_y, orientation_z, orientation_w,
               angular_velocity_x,  angular_velocity_y, angular_velocity_z,
               linear_acceleration_x, linear_acceleration_y, linear_acceleration_z):
        _id = 'Imu'
        imu_message = cls(id=_id, seq = seq, secs = secs, nsecs = nsecs, frame_id = frame_id, 
            orientation_x = orientation_x, orientation_y = orientation_y, orientation_z = orientation_z, orientation_w = orientation_w,
            angular_velocity_x = angular_velocity_x, angular_velocity_y = angular_velocity_y, angular_velocity_z = angular_velocity_z,
            linear_acceleration_x = linear_acceleration_x, linear_acceleration_y = linear_acceleration_y, linear_acceleration_z = linear_acceleration_z)
        # update the local storage content
        StreamState.stream[_id] = imu_message
        return imu_message
    
class UpdateImuMessage(graphene.Mutation):
    class Arguments:
        id = graphene.ID()
        seq = graphene.Int()
        secs = graphene.Int()
        nsecs = graphene.Int()
        frame_id = graphene.String()
        orientation_x = graphene.Float()
        orientation_y = graphene.Float()
        orientation_z = graphene.Float()
        orientation_w = graphene.Float()
        angular_velocity_x = graphene.Float()
        angular_velocity_y = graphene.Float()
        angular_velocity_z = graphene.Float()
        linear_acceleration_x = graphene.Float()
        linear_acceleration_y = graphene.Float()
        linear_acceleration_z = graphene.Float()

    ok = graphene.Boolean()
    imu_message = graphene.Field(lambda: ImuMessage)
    
    def mutate(self, info = None, **kwargs):
        imu_message = ImuMessage.create(**kwargs)
        ok = True
        # notify subscribers of an update
        Subscriptions.stream['Imu'].on_next(imu_message)
        return UpdateImuMessage(imu_message=imu_message, ok=ok)
### end Imu Message

### start NavSatFix Message
class NavSatFixMessage(graphene.ObjectType):
    class Meta:
        interfaces = (TelemMessage, )
    status_status = graphene.Int()
    status_service = graphene.Int()
    latitude = graphene.Float()
    longitude = graphene.Float()
    altitude = graphene.Float()
    # TODO: position_covariance array
    position_covariance_type = graphene.Int()
    
    @classmethod
    def create(cls, seq, secs, nsecs, frame_id, status_status, status_service, latitude, longitude, altitude, position_covariance_type):
        _id = 'NavSatFix'
        pose_stamped_message = cls(id=_id, seq = seq, secs = secs, nsecs = nsecs, frame_id = frame_id, status_status = status_status, status_service = status_service,
        latitude = latitude, longitude = longitude, altitude = altitude, position_covariance_type = position_covariance_type)
        # update the local storage content
        StreamState.stream[_id] = pose_stamped_message
        return pose_stamped_message
    
class UpdateNavSatFixMessage(graphene.Mutation):
    class Arguments:
        id = graphene.ID()
        seq = graphene.Int()
        secs = graphene.Int()
        nsecs = graphene.Int()
        frame_id = graphene.String()
        status_status = graphene.Int()
        status_service = graphene.Int()
        latitude = graphene.Float()
        longitude = graphene.Float()
        altitude = graphene.Float()
        # TODO: position_covariance array
        position_covariance_type = graphene.Int()

    ok = graphene.Boolean()
    nav_sat_fix_message = graphene.Field(lambda: NavSatFixMessage)
    
    def mutate(self, info = None, **kwargs):
        nav_sat_fix_message = NavSatFixMessage.create(**kwargs)
        ok = True
        # notify subscribers of an update
        Subscriptions.stream['NavSatFix'].on_next(nav_sat_fix_message)
        return UpdateNavSatFixMessage(nav_sat_fix_message=nav_sat_fix_message, ok=ok)
### end NavSatFix Message

class Mutation(graphene.ObjectType):
    update_state_message = UpdateStateMessage.Field()
    update_vfr_hud_message = UpdateVfrHudMessage.Field()
    update_pose_stamped_message = UpdatePoseStampedMessage.Field()
    update_nav_sat_fix_message = UpdateNavSatFixMessage.Field()
    update_imu_message = UpdateImuMessage.Field()

class Query(graphene.ObjectType):
    state_message = graphene.Field(StateMessage)
    vfr_hud_message = graphene.Field(VfrHudMessage)
    pose_stamped_message = graphene.Field(PoseStampedMessage)
    nav_sat_fix_message = graphene.Field(NavSatFixMessage)
    imu_message = graphene.Field(ImuMessage)
    
    params = graphene.List(ParameterBase)
    def resolve_params(self, info):
        tmp =[Parameters.params['testing'],Parameters.params['abcd']]
        print tmp
        return tmp
    
    def resolve_state_message(self, info):
        return StreamState.stream['State']
        
    def resolve_vfr_hud_message(self, info):
        return StreamState.stream['VfrHud']
        
    def resolve_pose_stamped_message(self, info):
        return StreamState.stream['PoseStamped']
    
    def resolve_nav_sat_fix_message(self, info):
        return StreamState.stream['NavSatFix']
    
    def resolve_imu_message(self, info):
        return StreamState.stream['Imu']
        
  
class Subscription(graphene.ObjectType):
    state_message = graphene.Field(StateMessage)
    vfr_hud_message = graphene.Field(VfrHudMessage)
    pose_stamped_message = graphene.Field(PoseStampedMessage)
    nav_sat_fix_message = graphene.Field(NavSatFixMessage)
    imu_message = graphene.Field(ImuMessage)

    def resolve_state_message(self, info):
        return Subscriptions.stream['State']
    
    def resolve_vfr_hud_message(self, info):
        return Subscriptions.stream['VfrHud']
        
    def resolve_pose_stamped_message(self, info):
        return Subscriptions.stream['PoseStamped']
    
    def resolve_nav_sat_fix_message(self, info):
        return Subscriptions.stream['NavSatFix']
        
    def resolve_imu_message(self, info):
        return Subscriptions.stream['Imu']
        

schema = graphene.Schema(query=Query, mutation=Mutation, subscription=Subscription, types=[ParameterFloat, ParameterInt])

state_message = StateMessage(
    id='State',
    seq = None, # graphene.Int()
    secs = None, # graphene.Int()
    nsecs = None, # graphene.Int()
    frame_id = None, # graphene.String()
    connected = None, # graphene.Boolean()
    armed = None, # graphene.Boolean()
    guided = None, # graphene.Boolean()
    mode = None, # graphene.String()
    system_status = None, # graphene.Int()
)

vfr_hud_message = VfrHudMessage(
    id = 'VfrHud',
    seq = None, # graphene.Int()
    secs = None, # graphene.Int()
    nsecs = None, # graphene.Int()
    frame_id = None, # graphene.String()
    airspeed = None, # graphene.Float()
    groundspeed = None, # graphene.Float()
    heading = None, # graphene.Int()
    throttle = None, # graphene.Float()
    altitude = None, # graphene.Float()
    climb = None, # graphene.Float()
)

pose_stamped_message = PoseStampedMessage(
    id = 'PoseStamped',
    seq = None, # graphene.Int()
    secs = None, # graphene.Int()
    nsecs = None, # graphene.Int()
    frame_id = None, # graphene.String()
    pose_position_x = None, # graphene.Float()
    pose_position_y = None, #  graphene.Float()
    pose_position_z = None, #  graphene.Float()
    pose_orientation_x = None, #  graphene.Float()
    pose_orientation_y = None, #  graphene.Float()
    pose_orientation_z = None, #  graphene.Float()
    pose_orientation_w = None, #  graphene.Float()
)

nav_sat_fix_message = NavSatFixMessage(
    id = 'NavSatFix',
    seq = None, # graphene.Int()
    secs = None, # graphene.Int()
    nsecs = None, # graphene.Int()
    frame_id = None, # graphene.String()
    status_status = None, # graphene.Int()
    status_service = None, # graphene.Int()
    latitude = None, # graphene.Float()
    longitude = None, # graphene.Float()
    altitude = None, # graphene.Float()
    position_covariance_type = None, # graphene.Int()
)

imu_message = ImuMessage(
    id = 'Imu',
    seq = None, # graphene.Int()
    secs = None, # graphene.Int()
    nsecs = None, # graphene.Int()
    frame_id = None, # graphene.String()
    orientation_x = None, # graphene.Float()
    orientation_y = None, # graphene.Float()
    orientation_z = None, # graphene.Float()
    orientation_w = None, #graphene.Float()
    angular_velocity_x = None, # graphene.Float()
    angular_velocity_y = None, # graphene.Float()
    angular_velocity_z = None, # graphene.Float()
    linear_acceleration_x = None, # graphene.Float()
    linear_acceleration_y = None, # graphene.Float()
    linear_acceleration_z = None, # graphene.Float()
)

ParameterFloat().create('testing', 123.45)
ParameterInt().create('abcd', 100001)

# init the state message
StreamState.stream['State'] = state_message
# init the VfrHud message
StreamState.stream['VfrHud'] = vfr_hud_message
# init the pose stamped message
StreamState.stream['PoseStamped'] = pose_stamped_message
# init the nav sat fix message
StreamState.stream['NavSatFix'] = nav_sat_fix_message
# init the imu message
StreamState.stream['Imu'] = imu_message

# add Subjects for all message types
Subscriptions.stream['State'] = Subject()
Subscriptions.stream['VfrHud'] = Subject()
Subscriptions.stream['PoseStamped'] = Subject()
Subscriptions.stream['NavSatFix'] = Subject()
Subscriptions.stream['Imu'] = Subject()