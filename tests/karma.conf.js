// an example karma.conf.js
module.exports = function(config) {
    config.set({
        basePath: '',
        frameworks: ['jasmine'],
        // list of files / patterns to load in the browser
        files: ['../*.js', '*.js']
    });
};