from __future__ import absolute_import
import graphene

from .data import get_character, get_droid, get_hero, get_human
from .data import get_state_message, set_state_message
from .data import setup

class Episode(graphene.Enum):
    NEWHOPE = 4
    EMPIRE = 5
    JEDI = 6

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
    

class Character(graphene.Interface):
    id = graphene.ID()
    name = graphene.String()
    friends = graphene.List(lambda: Character)
    appears_in = graphene.List(Episode)

    def resolve_friends(self, info):
        # The character friends is a list of strings
        return [get_character(f) for f in self.friends]

class Human(graphene.ObjectType):

    class Meta:
        interfaces = (Character, )
    home_planet = graphene.String()


class Droid(graphene.ObjectType):

    class Meta:
        interfaces = (Character, )
    primary_function = graphene.String()
    

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

    def mutate(self, info, id, seq, secs, nsecs, frame_id, connected, armed, guided, mode, system_status):
        state_message = StateMessage(id = id, seq = seq, secs = secs, nsecs = nsecs, frame_id = frame_id, 
        connected = connected, armed = armed, guided = guided, mode = mode, system_status = system_status)
        ok = True
        set_state_message(id, state_message)
        return UpdateStateMessage(state_message=state_message, ok=ok)

class Mutation(graphene.ObjectType):
    
    update_state_message = UpdateStateMessage.Field()


class Query(graphene.ObjectType):
    
    state_message = graphene.Field(StateMessage,
                           id=graphene.String()
                           )
    
    hero = graphene.Field(Character,
                          episode=Episode()
                          )
                          
    human = graphene.Field(Human,
                           id=graphene.String()
                           )
    droid = graphene.Field(Droid,
                           id=graphene.String()
                           )

    def resolve_hero(self, info, episode=None):
        return get_hero(episode)

    def resolve_human(self, info, id):
        return get_human(id)

    def resolve_droid(self, info, id):
        return get_droid(id)
    
    def resolve_state_message(self, info, id):
        return get_state_message(id)
  
# TODO: work this stuff out...      
# class Subscription(graphene.ObjectType):
#     class Meta:
#         description = 'Example subscription for TornadoQL'

#     onPost = graphene.Field(StateMessage)

#     def resolve_onPost(root, info):
#         return set_state_message

setup()

schema = graphene.Schema(query=Query, mutation=Mutation) # subscription=Subscription
