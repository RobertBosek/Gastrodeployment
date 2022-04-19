import requests
import json
from datetime import datetime as dt
import os
from bs4 import BeautifulSoup

CACHE_PATH = "../../../degginger_events"
FILENAME = ["eventslist",".json"]

class DeggingerEventService:

    def __init__(self, url, classtags={}):
        #self.classtags = classtags
        self.classtags = {
            'event_list': 'event-list',
            'list_item': 'event-list-item',
            'date_start' : "event-date-start",
            'date_end': 'event-date-end',
            'details': 'event-list-detail'
        }
        self.url = url
        self.base_url = url.split("/degginer")[0]

        d = dt.now()
        file = "{}/{}_{}_{}_{}{}".format(CACHE_PATH, FILENAME[0], d.year, d.month, d.day, FILENAME[1])

        print(os.getcwd())

        if os.path.isfile(file):
            print("no call")
            with open(file, 'r') as f:
                self.eventslist = json.load(f)
        else:
            self.eventslist = self.reload()

            '''
            #clear directory
            for f in os.listdir(CACHE_PATH):
                os.remove(os.path.join(CACHE_PATH, f))
            '''

            with open(file, 'w') as f:
                json.dump(self.eventslist, f)


        
    def reload(self):
        resp = requests.get(self.url)
        print("[DeggingerEventService]: calling {}".format(self.url))
        soup = BeautifulSoup(resp.text, 'html.parser')
        #find the eventlist within the HTML-Body
        eventlist = soup.body.find("div", {"class": self.classtags['event_list']})
        #find all single events in the list and return them as list of HTML-Elements
        events = eventlist.find_all("div", {"class": self.classtags['list_item']})
        return self.get_events(events)

    # get parsed information from the loaded list of HTML-Events
    def get_events(self, HTMLevents):

        eventslist = []
        for event in HTMLevents:
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

def main():
    test = DeggingerEventService('https://www.regensburg.de/degginger/programm')
    print("here")
    listed = test.eventslist
    for ev in listed:
        print(ev)

if __name__ == '__main__':
    main()