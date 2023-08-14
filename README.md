# ROSflight Documentation

This repository contains the source files for the documentation portion of the ROSflight website.

## Install mkdocs and LaTeX Support

This is easy:

``` bash
pip install mkdocs mkdocs-material pygments pymdown-extensions mike
```

(You don't have use to the global pip if you have python environments working, but for beginners, this is the simplest way to do it.)

## Run the mkdocs Server

Just type `mkdocs serve` in the root directory of the firmware repository. It should report to you something like:

``` bash
[I 170728 07:49:47 server:271] Serving on http://127.0.0.1:8000
[I 170728 07:49:47 handlers:58] Start watching changes
```

This means that mkdocs is hosting a webpage for you on http://127.0.0.1:8000. Navigate to that page in your web browser.

Now, as you make changes to the documentation, you should be able to see it on your browser. Just hit reload from time to time to see your changes.

## Adding Pages
To add a new page to the documentation, just take a look at the mkdocs.yml file in the root of the firmware directory. You should be able to figure it out from there.

## Adding LaTeX
The syntax for adding LaTeX math inline is `\( x \)`, which renders as \(x \). For adding a block, it's

``` latex
$$ E = mc^2 $$
```
which renders as
$$ E = mc^2 $$
erver by 

## Publishing changes onto the docs.rosflight.org website

To publish changes back onto the website, use the command `mike deploy --push [version]`, replacing `[version]` with the version of the documentation you want to push to. This will then update the build file on the `gh-pages` branch with the changes. Note that the gh-pages branch is protected, and this step needs to be done by someone in the ROSflight organization.

See https://github.com/jimporter/mike for more information on using mike.
