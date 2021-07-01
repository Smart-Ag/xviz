import sys, os, logging
import asyncio, json

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import xviz_avs
from xviz_avs.builder import XVIZBuilder, XVIZMetadataBuilder
from xviz_avs.server import XVIZServer, XVIZBaseSession

from scenarios.collector_scenario import CollectorScenario


class ScenarioSession(XVIZBaseSession):
    def __init__(self, socket, request, scenario=CollectorScenario()):
        super().__init__(socket, request)
        self._scenario = scenario
        self._socket = socket

    def on_connect(self):
        print("Connected!")
 
    def on_disconnect(self):
        print("Disconnect!")

    async def main(self):
        metadata = self._scenario.get_metadata()
        await self._socket.send(json.dumps(metadata))

        t = 0
        s = 0.001
        while True:
            try:
                message = self._scenario.get_message(t)
                await self._socket.send(json.dumps(message))

                t += 0.5
                await asyncio.sleep(s)
            except KeyboardInterrupt:
                s = 0.2

class ScenarioHandler:
    def __init__(self):
        pass

    def __call__(self, socket, request):
        return ScenarioSession(socket, request)

if __name__ == "__main__":
    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(logging.DEBUG)
    logging.getLogger("xviz-server").addHandler(handler)
    print("adding the handler...")

    server = XVIZServer(ScenarioHandler(), port=8081)
    loop = asyncio.get_event_loop()
    loop.run_until_complete(server.serve())
    loop.run_forever()
