from __future__ import absolute_import
import graphene
from rx.subjects import Subject

class StreamState:
    stream={} # this dict holds the current telem message data for each message type

class Subscriptions:
    stream={} # this dict holds Subject() for each message type

class TelemMessage(graphene.Interface):
    id = graphene.ID()
    seq = graphene.Int()
    secs = graphene.Int()
    nsecs = graphene.Int()
    frame_id = graphene.String()
    
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


class Mutation(graphene.ObjectType):
    update_state_message = UpdateStateMessage.Field()


class Query(graphene.ObjectType):
    state_message = graphene.Field(StateMessage)
    
    def resolve_state_message(self, info):
        return StreamState.stream['State']
  
  
class Subscription(graphene.ObjectType):
    state_message = graphene.Field(StateMessage)

    def resolve_state_message(self, info):
        # return the Subject for this message type
        return Subscriptions.stream['State']

schema = graphene.Schema(query=Query, mutation=Mutation, subscription=Subscription)

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

# init the state message
StreamState.stream['State'] = state_message

# add a Subject() for the state message
Subscriptions.stream['State'] = Subject()
