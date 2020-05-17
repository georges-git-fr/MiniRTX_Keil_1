document.addEventListener('DOMContentLoaded', function() {
    // cookie functions
    // based on http://www.quirksmode.org/js/cookies.html
    function create(name, value, days, path, domain) {
        path = typeof path !== 'undefined' ? path : '/';
        domain = typeof domain !== 'undefined' ? '; domain=' + domain : '';
        if (days) {
            var date = new Date();
            date.setTime(date.getTime() + (days * 24 * 60 * 60 * 1000));
            var expires = "; expires=" + date.toGMTString();
        } else {
            var expires = "";
        }
        document.cookie = name + "=" + value + expires + domain + "; path=" + path;
    }

    function read(name) {
        var nameEQ = name + "=";
        var ca = document.cookie.split(';');
        for (var i = 0; i < ca.length; i++) {
            var c = ca[i];
            while (c.charAt(0) === ' ') c = c.substring(1, c.length);
            if (c.indexOf(nameEQ) === 0) return c.substring(nameEQ.length, c.length);
        }
        return null;
    }

    /* Finds the shortest domain that we can set cookies on */
    function getShortestCookieDomain(hostname) {
        var restrictedHostnames = [".com", ".uk", ".co.uk", ".herokuapp.com"]

        var hostnameComponents = hostname.split(".")
        for (var i = hostnameComponents.length - 1; i > -1; i--) {
            var domain = "." + hostnameComponents.slice(i).join(".");
            if (restrictedHostnames.indexOf(domain) === -1) {
                return domain;
            }
        }
    }

    /*
    // Simple test runner for getDomain
    function assert(result, expected, description) {
        if (!(result === expected)) {
            throw "Assertion failed: " + description + ". (" + result + " != " + expected + ")"
        }
    }

    function getShortestCookieDomain() {
        assert(getShortestCookieDomain('localhost'), '.localhost', 'Hostname is localhost');
        assert(getShortestCookieDomain('pelion.com'), '.pelion.com', 'Hostname without www');
        assert(getShortestCookieDomain('www.pelion.com'), '.pelion.com', 'Hostname with www');
        assert(getShortestCookieDomain('www.pelion.co.uk'), '.pelion.co.uk', 'Extension with more than one dot');
        assert(getShortestCookieDomain('pelion.herokuapp.com'), '.pelion.herokuapp.com', 'Restricted host name');
    }
    */

    var cookieDomain = getShortestCookieDomain(window.location.hostname);
    create('testCookies', 'yes', 365, '/', cookieDomain);
    if (read("testCookies") === 'yes' && read("acceptCookies") !== 'yes') {
        document.getElementById('cookiemodule').style.display = 'block';

        document.getElementById("acceptcookie").addEventListener('click', function() {
            create('acceptCookies', 'yes', 365, '/', cookieDomain);
            document.getElementById('cookiemodule').style.display = "none";
        }, false);
    }
});
