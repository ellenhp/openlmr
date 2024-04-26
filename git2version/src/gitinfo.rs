use core::option::Option;

#[derive(Clone, Copy, PartialEq, Eq)]
pub struct TagInfo<'a> {
    /// The name of the tag
    pub tag: &'a str,

    /// The number of commits since the tag
    pub commits_since_tag: u32,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub struct GitInfo<'a, 'b> {
    /// Information on the tag that is the closest ancestor tag to the current commit.
    /// tag_info can be `None` if the repository doesn't have any tags or is a shallow clone and tags aren't available
    pub tag_info: Option<TagInfo<'a>>,

    /// ID of the current commit
    pub commit_id: &'b str,

    /// Whether the working directory was modified or whether the build was done from a clean working directory
    pub modified: bool,
}
