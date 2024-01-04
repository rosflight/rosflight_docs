# ROSflight Documentation

[![Documentation](https://github.com/rosflight/rosflight_documentation/actions/workflows/docs.yml/badge.svg)](https://github.com/rosflight/rosflight_documentation/actions/workflows/docs.yml) [![Mike build and deploy](https://github.com/rosflight/rosflight_docs/actions/workflows/mike-deploy.yml/badge.svg)](https://github.com/rosflight/rosflight_docs/actions/workflows/mike-deploy.yml)

This repository contains the source files for the documentation portion of the ROSflight website.

## Install mkdocs and LaTeX Support

``` bash
pip install -r requirements.txt
```

## Run the mkdocs Server

Type `mkdocs serve` in the root directory of the firmware repository. It should report to you something like:

``` bash
[I 170728 07:49:47 server:271] Serving on http://127.0.0.1:8000
[I 170728 07:49:47 handlers:58] Start watching changes
```

This means that mkdocs is hosting a webpage for you on http://127.0.0.1:8000. Navigate to that page in your web browser.

Now, as you make changes to the documentation, you should be able to see it on your browser. Just hit reload from time to time to see your changes.

## Adding Pages
To add a new page to the documentation, just take a look at the mkdocs.yml file in the root of the firmware directory. You should be able to figure it out from there.

## Adding LaTeX
The syntax for adding LaTeX math inline is `\( x \)`, which renders as x. For adding a block, it's

``` latex
$$ E = mc^2 $$
```

## Publishing changes onto the docs.rosflight.org website

Versioning is handled with the Python utility mike. See https://github.com/jimporter/mike for more information on using mike.

New versions of documentation will need to be deployed and pushed manually for changes to be reflected on the webpage. git-main and other git documentation versions will update automatically when their respective branches are updated.
