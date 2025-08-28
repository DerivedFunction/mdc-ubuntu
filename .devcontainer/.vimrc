" Enable type file detection. Vim will be able to try to detect the type of file in use.
filetype on

" Enable plugins and load plugin for the detected file type.
filetype plugin on

" Load an indent file for the detected file type.
filetype indent on

" Turn syntax highlighting on.
syntax on

" Add numbers to each line on the left-hand side.
set number

" Comment Color
hi Comment ctermfg=LightBlue

" Set tab spaces to two
set tabstop=2
set shiftwidth=2
set expandtab

" Max autocompletion suggestions
set pumheight=8

" There are certain files that we would never want to edit with Vim.
" Wildmenu will ignore files with these extensions.
set wildignore=*.docx,*.jpg,*.png,*.gif,*.pdf,*.pyc,*.exe,*.flv,*.img,*.xlsx


" PLUGINS ---------------------------------------------------------------- {{{


call plug#begin('~/.vim/plugged')

" auto complete () {} [] <></> but when you press Enter, you must Esc
  Plug 'DerivedFunction/pear-tree'

" Press TAB to see autocomplete menu
  Plug 'ervandew/supertab'

" File explorer
  Plug 'preservim/nerdtree'

" Airline theme and tab viewing support
  Plug 'vim-airline/vim-airline'

call plug#end()

" }}}


" MAPPINGS --------------------------------------------------------------- {{{

" Tab navigation like Firefox: only 'open new tab' works in terminal
nnoremap <C-t>     :tabnew<CR>
inoremap <C-t>     <Esc>:tabnew<CR>

" Close tabs
map <C-w> <Nop>
nnoremap <C-w>     :tabclose<CR>
inoremap <C-w>     <Esc>:tabclose<CR>
" Close tabs without saving
nnoremap <A-w>     :tabclose!<CR>
inoremap <A-w>     <Esc>:tabclose!<CR>

" Move between tabs
nnoremap <C-Left> 	:tabprevious<CR>
inoremap <C-Left>     	<Esc>:tabprevious<CR>
nnoremap <C-Right> 	:tabnext<CR>
inoremap <C-Right>     	<Esc>:tabnext<CR>

" Opens NERDTree by default Ctrl O
map <C-o> <Nop>
nnoremap <C-o>     :NERDTree<CR>
inoremap <C-o>     <Esc>:NERDTree<CR>


" Use CTRL-S for saving, also in Insert mode (<C-O> doesn't work well when
" using completions).
noremap <C-S>		:update<CR>
vnoremap <C-S>		<C-C>:update<CR>
inoremap <C-S>		<Esc>:update<CR>gi



" }}} 


"  VIMSCRIPT -------------------------------------------------------------- {{{

" Customize Terminal display (Do :Tm)
command Tm :cd %:p:h | :bel ter ++rows=8
command Vtm :cd %:p:h | :vert ter
command Open :NERDTree

"Disable default explorer and use NERDTREE
let g:loaded_netrw       = 1
let g:loaded_netrwPlugin = 1
let g:termdebug_wide=1
let NERDTreeQuitOnOpen=1

let g:airline#extensions#tabline#enabled = 1
packloadall

" STATUS LINE ------------------------------------------------------------ {{{

" }}}