try:
    from IPython import embed
except ImportError:
    import code

    def embed():
        vars = globals()
        vars.update(locals())
        shell = code.InteractiveConsole(vars)
        shell.interact()
# the above is not needed - the only purpose is to have a nice way
# of pausing the code execution in this simple example

from opcua import Client  # import the Client class


class SubHandler(object):
    """
    Subscription handler handles subscriptions (e.g. receives data change events).
    """

    def datachange_notification(self, node, val, data):
        """
        Subscription handler can be any object as long as it
        has this method with these arguments.
        """
        print(f"The data of the node {node} has changed to {val}")

    def event_notification(self, event):
        print("New event received: ", event)


if __name__ == '__main__':
    client = Client("opc.tcp://localhost:40842")  # instantiate client object; make sure address and port are correct
    try:  # technically, try..except does not need to be used, it is just safer
        client.connect()  # connect to server
        root = client.nodes.root  # get the root entity
        print("Root node is: ", root)
        objects = client.nodes.objects  # get the entity usually containing the objects
        print("Objects node is: ", objects)

        # Retreive some objects and variables
        # obj = objects.get_child("0:MyObject")  # this is the same as below
        obj = root.get_child(["0:Objects", "0:MyObject"])  # this addresses the object directly from root
        var1 = obj.get_child("1:Var1")  # retreive the first variable
        var2 = obj.get_child("1:Var2")  # retreive the second variable

        print("MyObject is: ", obj)
        print("Value of Var1 is: ", var1.get_value())
        print("Value of Var2 is: ", var2.get_value())
        # v = var2.get_value() + 1
        # var2.set_value(5)  # increments the current value by 1  <- this throws an error
        print("Value of Var2 after change is: ", var2.get_value())

        # the following creates "listener" for a specific variable (in this case "Var1")
        # create instance of a subscriber object (could be anything, just needs to have "datachange_notification" method)
        msclt = SubHandler()
        sub = client.create_subscription(500, msclt)  # create subscription
        handle = sub.subscribe_data_change(var1)  # set subscription for "data change of variable var1"

        embed()  # this just pauses the code be entering IPython console, type "quit()" to quit or press ctrl+D

        sub.unsubscribe(handle)  # cancel subscription
        sub.delete()
    finally:
        client.disconnect()  # disconnect
