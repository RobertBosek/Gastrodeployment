# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#### importing class DeggingerEventService from DeggingerEventService.py didnt work, as soon as it works delete
#### codeblock up to from DeggingerEventService import DeggingerEventService
import requests
from bs4 import BeautifulSoup


class DeggingerEventService:

    def __init__(self, url, classtags={}):
        # self.classtags = classtags
        self.classtags = {
            'event_list': 'event-list',
            'list_item': 'event-list-item',
            'date_start': "event-date-start",
            'date_end': 'event-date-end',
            'details': 'event-list-detail'
        }
        self.url = url
        self.base_url = url.split("/degginer")[0]
        self.HTMLeventslist = self.reload()

    def reload(self):
        resp = requests.get(self.url)
        soup = BeautifulSoup(resp.text, 'html.parser')
        # find the eventlist within the HTML-Body
        eventlist = soup.body.find("div", {"class": self.classtags['event_list']})
        # find all single events in the list and return them as list of HTML-Elements
        events = eventlist.find_all("div", {"class": self.classtags['list_item']})
        return events

    # get parsed information from the loaded list of HTML-Events
    def get_events(self):

        eventslist = []
        for event in self.HTMLeventslist:
            eventdict = self._extractInfo(event)
            eventslist.append(eventdict)
        return eventslist

    # extract the infomation within HTML-tags using classtags list, and return a single event as python dictionary
    def _extractInfo(self, html_event):

        event = {}
        startdate = html_event.find("div", {"class": self.classtags['date_start']})
        startdate = startdate('span')[0].text + " " + startdate('span')[1].text
        event['date_start'] = startdate

        enddate = html_event.find("div", {"class": self.classtags['date_end']})
        if enddate != None:
            enddate = enddate('span')[0].text + " " + enddate('span')[1].text
            event['date_end'] = enddate

        details = html_event.find("div", {"class": self.classtags['details']})
        event['title'] = details.find("a").text
        event['link'] = self.base_url + details.find("a")['href']
        event['text'] = details.find("p").text

        metahelper = details.find("span").text.split("\n")
        metahelper = metahelper[0] + "\n" + metahelper[-1].split("  ")[-1]
        if "Degginer" in metahelper:
            metahelper = metahelper.split("Degginger")
            metahelper = metahelper[0] + "\nDegginger" + metahelper[1]
        event['meta'] = metahelper
        return event



from .DeggingerEventService import DeggingerEventService
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

# TODO: use different msg type as soon as design layout of "events-app" is set
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        self.event_service = DeggingerEventService('https://www.regensburg.de/degginger/programm')

        self.publisher_ = self.create_publisher(String, '/degginer_events', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = str(self.event_service.get_events())
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
