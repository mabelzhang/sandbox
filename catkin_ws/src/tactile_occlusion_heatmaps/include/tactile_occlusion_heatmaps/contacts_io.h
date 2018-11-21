// Mabel Zhang
// 4 Oct 2018
//
// Reads contacts YAML file outputted by grasp_collect.py.
//

#ifndef _CONTACTS_IO_H_
#define _CONTACTS_IO_H_

class ContactsYaml
{
private:

  std::vector <std::string> objects_;


public:

  ContactsYaml (const std::string path)
  {
    if (! boost::filesystem::exists (path))
    {
      fprintf (stderr, "%sERROR: YAML file not found: %s%s\n", FAIL,
        path.c_str (), ENDC);
      return;
    }

    fprintf (stderr, "%sLoading contacts YAML %s%s\n", OKCYAN, path.c_str (),
      ENDC);
    YAML::Node config = YAML::LoadFile (path);
    YAML::Node objects_node = config ["objects"];

    objects_.resize (objects_node.size ());
    for (std::size_t i = 0; i < objects_node.size (); i++)
    {
      objects_.push_back (objects_node [i].as <std::string> ());
      std::cerr << objects_ [i].c_str () << std::endl;
    }
  }

  void get_objects (std::vector <std::string> & rv)
  {
    rv = std::vector <std::string> (objects_);
  }
};

#endif
