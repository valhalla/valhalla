# Add Mapzen Turn-by-Turn routing to a map

Mapzen Turn-by-Turn, which is a routing service powered by the Valhalla engine, adds routing and navigation to web or mobile applications. The service works globally, and provides dynamic and customizable routing by driving, walking, using multimodal and transit options, or bicycling, with clear directions for maneuvers along the route. In this walkthrough, you will learn how to make a map featuring Mapzen Turn-by-Turn. The map you create will provide:

- a route line between map locations (also known as waypoints)
- a text narrative of maneuvers to perform on the route
- distances along your route and estimated travel times
- functionality to drag the route start and endpoints to get a different path
- the ability change the mode of transportation, such as automobile, bicycle, pedestrian, or multimodal

## Get ready for the tutorial

In this tutorial, you will be planning a family [vacation](https://en.wikipedia.org/wiki/National_Lampoon%27s_Vacation) for travel by car from your home of Chicago, Illinois to visit a popular theme park in Anaheim, California. In your code, you will enter the start and end points of your trip and Mapzen Turn-by-Turn will calculate the route.

To complete the tutorial, you should have some familiarity with HTML and JavaScript, although all the source code is provided. You also need a Mapzen API key, which you can get by following the steps in the Mapzen [developer overview](https://mapzen.com/documentation/overview/).

You can use any text editor and operating system, but must maintain an Internet connection while you are working. The [Mapzen developer guide](https://mapzen.com/documentation/guides/install-text-editor/) has recommendations for text editors. The tutorial should take about an hour to complete.

## Create an index page

You are ready to start building your map.

1. Start your text editor with a blank document and copy and paste the following HTML. (Note: If the text editor you are using requires you to name and save a document at the time when it is first created, call the file `index.html`.)
2. Add the basic HTML tags, including `<!DOCTYPE HTML>`, `<html>`, `<head>`, and `<body>`. Your HTML might look like this:

    ```html
    <!DOCTYPE html>
    <html>
    <head>
    </head>
    <body>
    </body>
    </html>
    ```

    These form the basic structure of an HTML document. `<!DOCTYPE html>` goes at the top of every HTML page and indicates that it is written for HTML5, and the `<html>` tags tell your browser that the content is HTML. The `<head>` tag contains the title for the page and other metadata about the page, while the `<body>` is where you add the code and the rest of the content on your page. There are many [web tutorials](http://www.w3schools.com/html/default.asp) available to help you experiment with and learn more about HTML documents and the tags in them.

3. In the `<head>` tag, add a title, such as `<title>My Routing Map</title>`.
4. On the next line, add a metadata tag so you can properly display diacritics and characters from different languages.

    ```html
    <meta charset="utf-8">
    ```

5. Name your the document `index.html` (where the file name is `index` and the type is `.html`) and save it.

Your HTML should look like this:
```html
<!DOCTYPE html>
<html>
<head>
  <title>My Routing Map</title>
  <meta charset="utf-8">
</head>
<body>
</body>
</html>
```
6. Drag your index.html file onto a web browser tab. It should show your title, `My Routing Map`, but the web page canvas will be blank.

![Browser tab with title](/images/local-browser.png)

## Add references to CSS and JavaScript files

A cascading style sheet (CSS) is used to style a webpage, including layout and fonts, and JavaScript adds functionality to the page. In your `index.html` file, you need to list the CSS and JavaScript files needed to build your page.

When you request a route from Mapzen Turn-by-Turn, you are sending and receiving [JSON](https://en.wikipedia.org/wiki/JSON), which is a human-readable text format. This JSON can then be drawn on a map and shown as instructions for maneuvers along the route. The [Leaflet JavaScript library](http://leafletjs.com/) provides tools for building an interactive map for web and mobile devices. Leaflet is extensible, and developers have built additional tools for Leaflet maps.

You will reference the [mapzen.js library](https://www.mapzen.com/documentation/mapzen-js/), which simplifies the process of using Mapzen's maps within Leaflet. Mapzen.js contains all the Leaflet functionality, as well as additional tools for working with Mapzen maps.

You are linking to these CSS and JS files from a remote website, rather than from a file on your machine. You can also download the source files or install them through a package manager if you prefer to use a local copy.

1. In index.html, in the `<head>` section, add a reference to the Mapzen CSS file.

    ```html
    <link rel="stylesheet" href="https://mapzen.com/js/mapzen.css">
    ```

2. In the `<body>` section, add the mapzen.js JavaScript file.

    ```html
    <script src="https://mapzen.com/js/mapzen.js"></script>
    ```

7. Save your edits and refresh the browser.

After adding these, your index.html file should look something like this. Note that JavaScript can be inserted in either the `<head>` or the `<body>`, but the `<body>` may improve loading of the page.

```html
<!DOCTYPE html>
<html>
<head>
  <title>My Routing Map</title>
  <meta charset="utf-8">
  <link rel="stylesheet" href="https://mapzen.com/js/mapzen.css">
</head>
<body>
  <script src="https://mapzen.com/js/mapzen.js"></script>
</body>
</html>
```

At this point, your browser page is still empty. As you are working, it’s a good idea to save your edits and periodically reload the browser page. This helps you identify problems quicker and trace them back to your most recent changes. If you open the developer tools console in your browser and see a 404 error, it often means that the file cannot be found. You should make sure the paths in your HTML are correct before you continue further.

## Add a map to the page

To display a map on a page, you need a `<div>` element with an ID value, as well as a size for the box containing the map. If you want to know more about initializing a map, see the [mapzen.js documentation](https://mapzen.com/documentation/mapzen-js/get-started/).

1. At the bottom of the `<head>` section, add a `<style>` tag and the following size attributes to set the size of the map.

    ```html
    <style>
      #map {
        height: 100%;
        width: 100%;
        position: absolute;
      }
      html,body{margin: 0; padding: 0;}
    </style>
    ```

2. At the top of the `<body>` section, add the `<div>`.

    ```html
    <div id="map"></div>
    ```

3. Directly after the other `<script>` references, add this JavaScript code within a `<script>` tag to initialize a map and set your [API key](https://mapzen.com/documentation/overview/).

    ```html
    <script>
      L.Mapzen.apiKey = "your-mapzen-api-key";

      var map = L.Mapzen.map("map", {
        center: [41.8758,-87.6189],
        zoom: 16
      });
    </script>
    ```

    `L.xxxxx` is a convention used with the Leaflet API. To make sure the scripts load in the proper order, this code must be placed after the dependencies.

    The `center: [41.8758,-87.6189]` parameter sets the center point of the map, in decimal degrees, in Chicago, Illinois.

    The next line sets the `zoom` level, which is like a map scale or resolution, where a smaller value shows a larger area in less detail, and a larger zoom level value depicts smaller area in great detail.

4. Save your edits and refresh the browser. You should see a gray canvas with zoom controls and a Leaflet attribution in the bottom corner.

    ![Initial map showing Chicago area](/images/browser-initial-map.png)

Your `<body>` section should look like this:

```html
[...]
<body>
  <div id="map"></div>
  <script src="https://mapzen.com/js/mapzen.js"></script>
  <script>
    L.Mapzen.apiKey = "your-mapzen-api-key";

    var map = L.Mapzen.map("map", {
      center: [41.8758,-87.6189],
      zoom: 16
    });
  </script>
</body>
[...]
```

If your map is not loading properly, first check the browser console for status messages and resolve any 404 errors. You can also copy the example source code at the end of the section in case you mistyped any of the steps. If your project worked until now, ensure that your browser has WebGL support enabled (although it is unusual for it to be turned off) or turn on the developer tools in your browser to see if you can debug further. If you are still having trouble, [contact Mapzen](https://mapzen.com/documentation/overview/support/) or add an issue to the [documentation GitHub repository](https://github.com/valhalla/valhalla-docs/issues) so it can be investigated.

## Add waypoints for routing

So far, you have referenced the necessary files and initialized a map on the page. Now, you are ready to add the routing code to your page.

In the simplest implementation, your map will not provide the ability to search for places through geocoding or inputting coordinates otherwise. Therefore, you need to set the waypoints in your code. As you add functionality to your web page, you can set the initial coordinates through user interaction.

1. Add `//` at the beginning of the `center: [41.8758,-87.6189],` line to comment out that code. You no longer need to set the extent manually like this because the routing environment will be specifying it.

    ```js
    //center: [41.8758,-87.6189],
    ```

2. Inside the `<script>` tag, but after the closing `});` you added in the previous section, initialize routing with the following code. You can substitute your own coordinates for the start and end locations of the routing. These coordinates take you from Chicago, Illinois, to the entrance gates of the theme park in Anaheim, California.

    ```js
    var routingControl = L.Mapzen.routing.control({
      waypoints: [
        L.latLng(41.8758,-87.6189),
        L.latLng(33.8128,-117.9259)
      ],
      router: L.Mapzen.routing.router({costing:"your-routing-mode"}),
      summaryTemplate:'<div class="start">{name}</div><div class="info {costing}">{distance}, {time}</div>',
      routeWhileDragging: false
    }).addTo(map);
    ```

    By including a `summaryTemplate`, the directions can include totals of the length and expected time en route. Note that the `router:` has a placeholder for `"your-routing-mode"` that you will update these in the next steps.

3. Update the transportation mode `{costing:"your-routing-mode"}` to `{costing:"auto"}` to perform routing by automobile, again maintaining the quotation marks.

    ```js
    router: L.Mapzen.routing.router({costing:"auto"}),
    ```

5. Save your edits and refresh the browser. You should see a map, the route line, and icons and summary text in the narration box.

    ![Map showing Mapzen Turn-by-Turn route and directions](/images/route-map-valhalla.png)

The `<body>` section should look something like this, but with your own API key for the `router`:

```html
[...]
<body>
  <div id="map"></div>
  <script src="https://mapzen.com/js/mapzen.js"></script>
  <script>
    L.Mapzen.apiKey = "your-mapzen-api-key";

    var map = L.Mapzen.map("map", {
      //center: [41.8758,-87.6189],
      zoom: 16
    });

    var routingControl = L.Mapzen.routing.control({
      waypoints: [
        L.latLng(41.8758,-87.6189),
        L.latLng(33.8128,-117.9259)
      ],
      router: L.Mapzen.routing.router({costing:"auto"}),
      summaryTemplate:'<div class="start">{name}</div><div class="info {costing}">{distance}, {time}</div>',
      routeWhileDragging: false
    }).addTo(map);
  </script>
</body>
[...]
```

Mapzen Turn-by-Turn also provides the ability to specify additional waypoints through which your route should pass, such as visiting family in Kansas and Arizona and bobbing your head at Grand Canyon National Park. Currently, you can drag the start and end points (and add waypoints in between) to update the routing, but the route will not be recalculated until you drop the points. On your own, you can set the option for `routeWhileDragging` to `true` if you want to update the route while moving points on the map, although this can be slow and costly to make many queries.

## Change the route line color

The symbols for the map are defined in the basemap, but the route line may be hard to distinguish from the roads in your map. You can use `L.routing.control` to update the color of the route line.

1. After the closing line of the `waypoints:` block and immediately before the `router:` block, insert the following source code:

    ```js
    lineOptions: {
      styles: [ {color: "white",opacity: 0.8, weight: 12},
              {color: "#2676C6", opacity: 1, weight: 6}
    ]},
    ```

2. Save your edits and refresh the browser. The line should look thicker than before.

    ![Map showing updated route line color](/images/route-map-valhalla-line-color.png)

3. Click through the points in the directions list to pan and zoom the map to the location of each maneuver, including your destination.

    ![Arrival at your destination](/images/route-map-valhalla-destination.png)

The completed HTML should look something like this:

```html
<!DOCTYPE html>
<html>
<head>
  <title>My Routing Map</title>
  <meta charset="utf-8">
  <link rel="stylesheet" href="https://mapzen.com/js/mapzen.css">
  <style>
    #map {
      height: 100%;
      width: 100%;
      position: absolute;
    }
      html,body{margin: 0; padding: 0;}
  </style>
</head>
<body>
  <div id="map"></div>
  <script src="https://mapzen.com/js/mapzen.js"></script>
  <script>
    L.Mapzen.apiKey = "your-mapzen-api-key";

    var map = L.Mapzen.map("map", {
      //center: [41.8758,-87.6189],
      zoom: 16
    });

    var routingControl = L.Mapzen.routing.control({
      waypoints: [
        L.latLng(41.8758,-87.6189),
        L.latLng(33.8128,-117.9259)
      ],
      lineOptions: {
        styles: [ {color: "white",opacity: 0.8, weight: 12},
                {color: "#2676C6", opacity: 1, weight: 6}
      ]},
      router: L.Mapzen.routing.router({costing:"auto"}),
      summaryTemplate:'<div class="start">{name}</div><div class="info {costing}">{distance}, {time}</div>',
      routeWhileDragging: false
    }).addTo(map);
  </script>
</body>
</html>
```

## Tutorial summary and next steps

In this tutorial, you learned the basics of making a map with Mapzen Turn-by-Turn routing. You can now take what you have learned and add more functionality to your map and embed it in your own projects. For example, you may want to add code to allow the user to pick routing locations with a button, change the costing mode used for routing, or set other options. Each of the routing modes Mapzen Turn-by-Turn supports has many options that can be used to influence the output route and estimated time. For example, automobile routing allows you to set penalties and costs to avoid toll roads or crossing international borders, and bicycle routing allows you to specify the category of bicycle so you are routed on appropriate paths for your equipment.

You can review the [documentation](/turn-by-turn/api-reference.md) to learn more about routing with Mapzen Turn-by-Turn.
